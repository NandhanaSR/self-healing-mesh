"""Tests for live_model.py (no boards needed).

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import queue
import threading
import unittest

from live_model import LiveMesh, PortStatus, SerialReader
from serial_parser import BoardStream, Neighbor, NodeTable, Route

T0 = 1_000_000.0


def table(node, nbrs=(), routes=()):
    return NodeTable(node, tuple(nbrs), tuple(routes))


def nbr(i, state="HEALTHY", ewma=-60.0, q_pred=0.9):
    return Neighbor(i, state, ewma, 0.0, 1.0, 0.0, int(q_pred * 100), q_pred)


def route(dest, nxt, prio=None, cost=1.0, hops=1):
    return Route(dest, nxt, cost, nxt if prio is None else prio, hops)


def feed_text(mesh, port, text, t=T0):
    s = BoardStream()
    for line in text.splitlines():
        if line.startswith("---- node"):
            t += 1.0
        for rec in s.feed(line):
            mesh.apply(port, t, rec)
    for rec in s.flush():
        mesh.apply(port, t, rec)
    return t


class ModelTests(unittest.TestCase):
    def test_table_replaces_snapshot_and_keeps_history(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, table(2, [nbr(1, ewma=-60), nbr(3, ewma=-70)], [route(1, 1), route(4, 3)]))
        m.apply("COM3", T0 + 1, table(2, [nbr(1, ewma=-61)], [route(1, 1)]))
        b = m.boards["COM3"]
        self.assertEqual(b.node_id, 2)
        self.assertEqual(set(b.neighbors), {1})
        self.assertEqual(set(b.routes), {1})
        self.assertEqual(list(b.ewma_hist[1]), [-60, -61])
        self.assertIn("route →N4 lost", m.events[-1][2])

    def test_route_change_logged_but_not_on_first_table(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, table(2, routes=[route(4, 3)]))
        self.assertEqual(len(m.events), 0)
        m.apply("COM3", T0 + 1, table(2, routes=[route(4, 1, prio=1, cost=3.0)]))
        lvl, msg = m.events[-1][1:]
        self.assertEqual(lvl, "ROUTE")
        self.assertIn("next N3 ⇒ N1", msg)

    def test_no_route_logged_once_until_recovered(self):
        from serial_parser import Forward, NoRoute
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, table(0))
        for i in range(5):
            m.apply("COM3", T0 + i, NoRoute(1))
        m.apply("COM3", T0 + 6, Forward(1, 2, False))
        b = m.boards["COM3"]
        self.assertEqual((b.no_route, b.fwd), (5, 1))
        self.assertEqual([e[1] for e in m.events], ["CRIT", "OK"])

    def test_dest_and_priority_unknown_until_printed(self):
        from serial_parser import DestChanged, PriorityChanged
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, table(0))
        b = m.boards["COM3"]
        self.assertIsNone(b.dest)
        self.assertIsNone(b.priority)
        m.apply("COM3", T0, DestChanged(4))
        m.apply("COM3", T0, PriorityChanged(True))
        self.assertEqual((b.dest, b.priority), (4, True))

    def test_duplicate_id_warning(self):
        m = LiveMesh(["COM3", "COM4"])
        m.apply("COM3", T0, table(0))
        m.apply("COM4", T0, table(0))
        self.assertEqual(m.events[-1][1], "WARN")
        self.assertIn("MY_ID 0", m.events[-1][2])

    def test_reboot_clears_tables(self):
        from serial_parser import Ready
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, table(2, [nbr(1)], [route(1, 1)]))
        m.apply("COM3", T0 + 1, Ready(2))
        b = m.boards["COM3"]
        self.assertEqual((b.neighbors, b.routes, b.table_time), ({}, {}, None))

    def test_stale(self):
        m = LiveMesh(["COM3"])
        b = m.boards["COM3"]
        self.assertTrue(b.stale(T0))
        m.apply("COM3", T0, table(1))
        self.assertFalse(b.stale(T0 + 2.0))
        self.assertTrue(b.stale(T0 + 3.5))

    def test_sample_capture(self):
        m = LiveMesh(["node2"])
        with open(__file__.replace("test_live_model.py", "samples/node2.txt"), encoding="utf-8") as f:
            feed_text(m, "node2", f.read())
        b = m.boards["node2"]
        self.assertEqual(b.node_id, 2)
        self.assertEqual(b.neighbors[3].state, "DEAD")
        self.assertNotIn(3, b.routes)
        self.assertEqual(b.priority, True)
        self.assertEqual((b.fwd, b.fwd_prio, b.delivered), (5, 2, 1))
        self.assertEqual(list(b.qpred_hist[3]), [0.84, 0.55, 0.38, 0.22, 0.0])
        levels = [e[1] for e in m.events]
        self.assertIn("WARN", levels)           # HEALTHY -> DEGRADING
        self.assertIn("ROUTE", levels)          # route to 4 moved from N3 to N1


class PathTests(unittest.TestCase):
    def setUp(self):
        # 0 -> 2 -> 1 normally; priority traffic 0 -> 3 -> 1. Board 4 has no route to 1.
        self.m = LiveMesh(["A", "B", "C", "D", "E"])
        self.m.apply("A", T0, table(0, routes=[route(1, 2, prio=3, hops=2), route(5, 4)]))
        self.m.apply("B", T0, table(2, routes=[route(1, 1)]))
        self.m.apply("C", T0, table(3, routes=[route(1, 1)]))
        self.m.apply("D", T0, table(1, routes=[route(0, 2)]))
        self.m.apply("E", T0, table(4))

    def test_normal_and_priority(self):
        self.assertEqual(self.m.path(0, 1), ([0, 2, 1], "ok"))
        self.assertEqual(self.m.path(0, 1, priority=True), ([0, 3, 1], "ok"))

    def test_self(self):
        self.assertEqual(self.m.path(1, 1), ([1], "ok"))

    def test_unknown_board(self):
        self.assertEqual(self.m.path(1, 0), ([1, 2], "no-route"))
        self.m.apply("D", T0, table(1, routes=[route(5, 5)]))
        self.assertEqual(self.m.path(1, 5), ([1, 5], "ok"))
        self.assertEqual(self.m.path(5, 1), ([5], "unknown"))    # board 5 not plugged in

    def test_no_route(self):
        self.assertEqual(self.m.path(0, 5), ([0, 4], "no-route"))

    def test_loop(self):
        self.m.apply("B", T0, table(2, routes=[route(1, 0)]))
        self.assertEqual(self.m.path(0, 1), ([0, 2, 0], "loop"))


class FakeSerial:
    """Hands out byte chunks, then raises like a pulled USB cable."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.written = b""
        self.closed = False
        self.done = threading.Event()

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, n):
        if not self.chunks:
            self.done.set()
            raise OSError("device disconnected")
        return self.chunks.pop(0)

    def write(self, data):
        self.written += data

    def close(self):
        self.closed = True


class ReaderTests(unittest.TestCase):
    def test_lines_split_across_reads(self):
        fake = FakeSerial([b"---- node 3 ----\r\n  route -> 1  next 1 (co", b"st 1.00) | priority next 1 ",
                           b"(hops 1)\r\n[FWD] no route to 5\r\n"])
        q = queue.Queue()
        r = SerialReader("COM9", q, opener=lambda port: fake)
        r.start()
        self.assertTrue(fake.done.wait(2))
        r.stop()
        r.join(3)
        recs = [q.get_nowait()[2] for _ in range(q.qsize())]
        self.assertEqual(recs[0], PortStatus(True))
        self.assertEqual(recs[1], table(3, routes=[route(1, 1)]))
        self.assertEqual(type(recs[2]).__name__, "NoRoute")
        self.assertFalse(recs[-1].connected)
        self.assertTrue(fake.closed)

    def test_open_failure_reported(self):
        q = queue.Queue()

        def busy(port):
            raise OSError("could not open port 'COM9': PermissionError(13, 'Access is denied.')")

        r = SerialReader("COM9", q, opener=busy)
        r.start()
        port, _, rec = q.get(timeout=2)
        r.stop()
        r.join(3)
        self.assertEqual(port, "COM9")
        self.assertFalse(rec.connected)
        self.assertIn("Access is denied", rec.detail)

    def test_send(self):
        fake = FakeSerial([])
        r = SerialReader("COM9", queue.Queue(), opener=lambda port: fake)
        self.assertFalse(r.send("p"))           # not open yet
        r._ser = fake
        self.assertTrue(r.send("4"))
        self.assertEqual(fake.written, b"4")


if __name__ == "__main__":
    unittest.main()

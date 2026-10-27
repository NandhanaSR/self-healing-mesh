"""Tests for the --demo pieces: simulated boards (demo_sim.py) and DEMO sensor data (demo_anomaly.py).

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import queue
import unittest

from demo_anomaly import ANOM_JUMP, ANOM_SPIKE, ANOM_STUCK, DemoAnomalies, Detector
from demo_sim import NODES, SimMesh
from live_model import LiveMesh, SerialReader
from serial_parser import BoardStream, LinkEvent, Unknown, parse_line


def run_sim(seconds, seed=1):
    """Step the simulation and feed every printed line through the real parser and model."""
    sim = SimMesh(seed=seed)
    mesh = LiveMesh(sim.ports)
    streams = {p: BoardStream() for p in sim.ports}
    lines = []
    for _ in range(int(seconds * 2)):
        sim.step()
        for n in NODES:
            port = f"SIM{n}"
            for line in sim.drain(n).splitlines():
                lines.append((n, sim.t, line))
                for rec in streams[port].feed(line):
                    mesh.apply(port, sim.t, rec)
    return sim, mesh, lines


class SimBoardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim, cls.mesh, cls.lines = run_sim(60)

    def test_every_line_is_a_firmware_line(self):
        bad = [l for _, _, l in self.lines if isinstance(parse_line(l), Unknown)]
        self.assertEqual(bad, [])

    def test_all_boards_report(self):
        self.assertEqual(sorted(self.mesh.by_id()), list(NODES))

    def test_link_predicted_to_fail_before_it_dies(self):
        evs = [(t, parse_line(l)) for n, t, l in self.lines if n == 2 and l.startswith("[LINK] node 1")]
        degrading = next(t for t, e in evs if isinstance(e, LinkEvent) and e.new == "DEGRADING")
        dead = next(t for t, e in evs if isinstance(e, LinkEvent) and e.new == "DEAD")
        self.assertLess(degrading, dead)

    def test_traffic_reroutes_and_reaches_the_sink(self):
        self.assertGreater(self.mesh.route_changes, 0)
        self.assertGreater(self.mesh.by_id()[1].delivered, 0)
        self.assertEqual(self.mesh.path(0, 1)[1], "ok")

    def test_route_away_from_degrading_link(self):
        # Replay the first 20 s: once N2->N1 is DEGRADING, N2's next hop to N1 should move to N3.
        _, mesh, _ = run_sim(20)
        self.assertEqual(mesh.path(2, 1)[0], [2, 3, 1])

    def test_keys_change_destination_and_priority(self):
        sim = SimMesh(seed=2)
        sim.drain(0)
        sim.boards[0].port.write(b"3p")
        self.assertEqual(sim.drain(0).splitlines(), ["destination = 3", "priority = ON"])
        self.assertEqual((sim.boards[0].dest, sim.boards[0].prio), (3, True))

    def test_through_serial_reader(self):
        sim = SimMesh(seed=3)
        q = queue.Queue()
        r = SerialReader("SIM0", q, opener=sim.open)
        r.start()
        try:
            kinds = set()
            while "NodeTable" not in kinds:
                kinds.add(type(q.get(timeout=5)[2]).__name__)
        finally:
            r.stop()
        self.assertIn("Ready", kinds)


class DetectorTests(unittest.TestCase):
    def feed(self, values):
        d = Detector()
        return [d.update(v) for v in values]

    def test_spike(self):
        flags = self.feed([2000, 2010, 1990, 2005, 1995, 2000, 3500])
        self.assertTrue(flags[-1] & ANOM_SPIKE)
        self.assertEqual(flags[:-1], [0] * 6)

    def test_jump(self):
        flags = self.feed([1000, 1900])
        self.assertTrue(flags[-1] & ANOM_JUMP)

    def test_stuck_after_20_same(self):
        flags = self.feed([1234] * 21)
        self.assertFalse(flags[19] & ANOM_STUCK)
        self.assertTrue(flags[20] & ANOM_STUCK)

    def test_demo_generator_produces_flags(self):
        a = DemoAnomalies(seed=4)
        for _ in range(300):
            a.step([0, 1])
        counts = {k: sum(s.counts[k] for pair in a.sensors.values() for s in pair) for k in ("SPIKE", "JUMP", "STUCK")}
        self.assertTrue(all(v > 0 for v in counts.values()), counts)


if __name__ == "__main__":
    unittest.main()

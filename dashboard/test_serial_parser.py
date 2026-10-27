"""Tests for serial_parser.py using sample lines (no boards needed).

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import unittest

from serial_parser import (BoardStream, DataRx, DestChanged, Forward, LinkEvent, Neighbor,
                           NodeHeader, NodeTable, NoRoute, PriorityChanged, Ready, Route,
                           Unknown, parse_line)

# Format strings copied verbatim from mesh_node.ino; C %u becomes %d for Python's % operator.
FMT_HEADER = "---- node %u ----\n"
FMT_NBR = "  nbr %u  %-9s ewma %.1f  slope %.2f  PRR %.2f dPRR %.2f  q %u%% q_pred %.2f\n"
FMT_ROUTE = "  route -> %u  next %u (cost %.2f) | priority next %u (hops %u)\n"
FMT_LINK = "[LINK] node %u: %s -> %s  (ewma %.1f dBm, slope %.2f dB/s, PRR %.2f, q %u%%, q_pred %.2f)\n"
FMT_FWD = "[FWD] dest %u via %u%s\n"
FMT_NOROUTE = "[FWD] no route to %u\n"
FMT_DATA = "[DATA] from %u  s1=%d s2=%d  anomaly=0x%02X/0x%02X%s\n"
FMT_DEST = "destination = %u\n"
FMT_PRIO = "priority = %s\n"
FMT_READY = "Node %u ready. Type 0-5 = destination, p = toggle priority.\n"


def c(fmt, *args):
    """Format like the ESP32's Serial.printf, with the CRLF a serial terminal may add."""
    return (fmt.replace("%u", "%d") % args).replace("\n", "\r\n")


# A hand-written capture from one board (MY_ID 2), including ROM boot noise.
RECORDED = """\
ets Jul 29 2019 12:21:46

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
Node 2 ready. Type 0-5 = destination, p = toggle priority.
---- node 2 ----
  nbr 1  HEALTHY   ewma -58.4  slope 0.12  PRR 1.00 dPRR 0.00  q 92% q_pred 0.91
  nbr 3  DEGRADING ewma -81.7  slope -1.85  PRR 0.70 dPRR -0.10  q 41% q_pred 0.38
  route -> 1  next 1 (cost 1.00) | priority next 1 (hops 1)
  route -> 3  next 1 (cost 2.00) | priority next 1 (hops 2)
[FWD] dest 1 via 1
[LINK] node 3: DEGRADING -> DEAD  (ewma -88.0 dBm, slope -2.40 dB/s, PRR 0.00, q 0%, q_pred 0.00)
[DATA] from 4  s1=1873 s2=-3  anomaly=0x00/0x1F  PRIORITY
---- node 2 ----
  nbr 1  HEALTHY   ewma -58.1  slope 0.10  PRR 1.00 dPRR 0.00  q 93% q_pred 0.92
  nbr 3  DEAD      ewma -88.0  slope -2.40  PRR 0.00 dPRR 0.00  q 0% q_pred 0.00
  route -> 1  next 1 (cost 1.00) | priority next 1 (hops 1)
"""


class ParseLineTests(unittest.TestCase):
    def test_header(self):
        self.assertEqual(parse_line(c(FMT_HEADER, 3)), NodeHeader(3))

    def test_neighbor_each_state_padding(self):
        # %-9s pads the state name, so the gap after it varies.
        for st in ("HEALTHY", "DEGRADING", "DEAD"):
            line = c(FMT_NBR, 4, st, -72.3, -0.85, 0.9, -0.05, 63, 0.47)
            self.assertEqual(parse_line(line), Neighbor(4, st, -72.3, -0.85, 0.9, -0.05, 63, 0.47))

    def test_route(self):
        line = c(FMT_ROUTE, 5, 1, 2.75, 3, 2)
        self.assertEqual(parse_line(line), Route(dest=5, next=1, cost=2.75, prio_next=3, hops=2))

    def test_link_event(self):
        line = c(FMT_LINK, 3, "HEALTHY", "DEGRADING", -79.5, -1.20, 0.80, 48, 0.44)
        self.assertEqual(parse_line(line),
                         LinkEvent(3, "HEALTHY", "DEGRADING", -79.5, -1.2, 0.8, 48, 0.44))

    def test_forward_normal_and_priority(self):
        self.assertEqual(parse_line(c(FMT_FWD, 1, 2, "")), Forward(1, 2, False))
        self.assertEqual(parse_line(c(FMT_FWD, 1, 2, " (PRIORITY)")), Forward(1, 2, True))

    def test_no_route(self):
        self.assertEqual(parse_line(c(FMT_NOROUTE, 5)), NoRoute(5))

    def test_data_ignores_sensor_fields(self):
        self.assertEqual(parse_line(c(FMT_DATA, 0, 4095, -12, 0xA5, 0x00, "")), DataRx(0, False))
        self.assertEqual(parse_line(c(FMT_DATA, 0, 0, 0, 0, 0, "  PRIORITY")), DataRx(0, True))

    def test_console_replies(self):
        self.assertEqual(parse_line(c(FMT_DEST, 4)), DestChanged(4))
        self.assertEqual(parse_line(c(FMT_PRIO, "ON")), PriorityChanged(True))
        self.assertEqual(parse_line(c(FMT_PRIO, "OFF")), PriorityChanged(False))
        self.assertEqual(parse_line(c(FMT_READY, 0)), Ready(0))

    def test_nan_values(self):
        rec = parse_line("  nbr 2  HEALTHY   ewma nan  slope nan  PRR 0.00 dPRR 0.00  q 0% q_pred nan")
        self.assertIsInstance(rec, Neighbor)
        self.assertNotEqual(rec.ewma, rec.ewma)        # nan

    def test_blank_and_unknown(self):
        self.assertIsNone(parse_line("\r\n"))
        self.assertIsNone(parse_line("   "))
        self.assertEqual(parse_line("ESP-NOW init FAILED"), Unknown("ESP-NOW init FAILED"))
        # A line cut off mid-way (e.g. port opened during printing) must not half-parse.
        self.assertIsInstance(parse_line("  nbr 1  HEALTHY   ewma -58.4  slope 0.1"), Unknown)
        self.assertIsInstance(parse_line("e -> 1  next 1 (cost 1.00) | priority next 1 (hops 1)"), Unknown)


class BoardStreamTests(unittest.TestCase):
    def feed_all(self, text):
        s = BoardStream()
        out = []
        for line in text.splitlines(keepends=True):
            out.extend(s.feed(line))
        out.extend(s.flush())
        return s, out

    def test_recorded_capture(self):
        s, out = self.feed_all(RECORDED)
        self.assertEqual(s.node_id, 2)
        kinds = [type(r).__name__ for r in out]
        self.assertEqual(kinds, ["Unknown", "Unknown", "Ready", "NodeTable", "Forward",
                                 "LinkEvent", "DataRx", "NodeTable"])

        first, second = out[3], out[7]
        self.assertEqual(first.node, 2)
        self.assertEqual([n.nbr for n in first.neighbors], [1, 3])
        self.assertEqual(first.neighbors[1].state, "DEGRADING")
        self.assertEqual(first.neighbors[1].q_pred, 0.38)
        self.assertEqual([r.dest for r in first.routes], [1, 3])

        # Route to 3 vanished: the firmware skips unreachable routes, so the snapshot
        # simply has no entry for it.
        self.assertEqual(second.neighbors[1].state, "DEAD")
        self.assertEqual([r.dest for r in second.routes], [1])
        self.assertEqual(out[6], DataRx(4, True))

    def test_empty_block(self):
        # A board with no neighbours yet prints only the header.
        _, out = self.feed_all(c(FMT_HEADER, 0) + c(FMT_HEADER, 0))
        self.assertEqual(out, [NodeTable(0), NodeTable(0)])

    def test_lines_before_first_header_are_dropped(self):
        text = (c(FMT_NBR, 1, "HEALTHY", -60.0, 0.0, 1.0, 0.0, 90, 0.9)
                + c(FMT_ROUTE, 1, 1, 1.0, 1, 1)
                + c(FMT_HEADER, 5)
                + c(FMT_ROUTE, 1, 0, 2.0, 0, 2))
        s, out = self.feed_all(text)
        self.assertEqual(out, [NodeTable(5, (), (Route(1, 0, 2.0, 0, 2),))])
        self.assertEqual(s.node_id, 5)

    def test_event_line_ends_block(self):
        s = BoardStream()
        self.assertEqual(s.feed(c(FMT_HEADER, 1)), [])
        self.assertEqual(s.feed(c(FMT_ROUTE, 0, 0, 1.0, 0, 1)), [])
        out = s.feed(c(FMT_NOROUTE, 4))
        self.assertEqual(out, [NodeTable(1, (), (Route(0, 0, 1.0, 0, 1),)), NoRoute(4)])
        # Table lines after the block closed are not attributed to the old block.
        self.assertEqual(s.feed(c(FMT_ROUTE, 2, 2, 1.0, 2, 1)), [])
        self.assertEqual(s.flush(), [])


if __name__ == "__main__":
    unittest.main()

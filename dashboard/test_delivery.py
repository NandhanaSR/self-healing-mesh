"""Tests for the Delivery % figure: DATA delivered / sent per sender, counted while both boards are live.

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import contextlib
import glob
import io
import os
import unittest
from unittest import mock

from rich.console import Console

from live_model import LiveMesh, PortStatus
from mesh_dashboard import DELIVERY_OK, DELIVERY_WARN, delivery_style, main, render
from serial_parser import DataRx, DestChanged, Forward, NodeTable, NoRoute

SAMPLES = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "samples", "mesh4", "node*.txt")))


class Bench:
    """Boards N0..N(n-1) on ports P0.. ; tick() prints every live board's table once a second."""

    def __init__(self, n=3):
        self.m = LiveMesh([f"P{i}" for i in range(n)])
        self.live = set(range(n))
        for i in range(n):
            self.m.apply(f"P{i}", 0.0, PortStatus(True))

    def tick(self, t):
        for i in sorted(self.live):
            self.m.apply(f"P{i}", t, NodeTable(i))

    def line(self, node, t, rec):
        self.m.apply(f"P{node}", t, rec)

    def pair(self, src, dest):
        return self.m.delivery.get((src, dest))


def draw(m, now, **kw):
    console = Console(file=io.StringIO(), width=170, height=50, record=True, color_system=None)
    console.print(render(m, now, **kw))
    return console.export_text()


class CountingTests(unittest.TestCase):
    def test_direct_link(self):
        b = Bench(2)
        for s in range(1, 11):
            b.tick(s)
            b.line(0, s + 0.1, Forward(1, 1, False))
            if s not in (4, 7):                     # two packets lost
                b.line(1, s + 0.2, DataRx(0, False))
        d = b.pair(0, 1)
        self.assertEqual((d.delivered, d.sent, round(d.pct)), (8, 10, 80))

    def test_no_route_counts_as_sent(self):
        b = Bench(2)
        for s in range(1, 6):
            b.tick(s)
            b.line(0, s + 0.1, NoRoute(1) if s <= 2 else Forward(1, 1, False))
            if s > 2:
                b.line(1, s + 0.2, DataRx(0, False))
        self.assertEqual((b.pair(0, 1).delivered, b.pair(0, 1).sent), (3, 5))

    def test_only_lines_towards_current_destination(self):
        b = Bench(3)
        for s in range(1, 6):
            b.tick(s)
            b.line(0, s + 0.1, Forward(1, 1, False))
            b.line(0, s + 0.2, Forward(2, 2, False))    # relaying someone else's packet to N2
            b.line(1, s + 0.3, DataRx(0, False))
        self.assertEqual(b.pair(0, 1).sent, 5)
        self.assertIsNone(b.pair(0, 2))

    def test_not_counted_before_identified(self):
        b = Bench(2)
        b.line(0, 0.5, Forward(1, 1, False))           # no table yet: board not identified
        b.line(1, 0.6, DataRx(0, False))
        self.assertIsNone(b.pair(0, 1))

    def test_paused_while_destination_stale_or_unplugged(self):
        b = Bench(2)
        for s in range(1, 6):
            b.tick(s)
            b.line(0, s + 0.1, Forward(1, 1, False))
            b.line(1, s + 0.2, DataRx(0, False))
        b.live.discard(1)                               # N1 stops printing tables (stuck)
        for s in range(6, 16):
            b.tick(s)
            b.line(0, s + 0.1, Forward(1, 1, False))
        b.m.apply("P0", 16.0, PortStatus(False, "port closed"))   # N0 unplugged
        b.line(0, 16.1, Forward(1, 1, False))
        d = b.pair(0, 1)
        # sends at 6.1 and 7.1 still count: N1's last table (t=5) is within STALE_S (3 s)
        self.assertEqual((d.delivered, d.sent), (5, 7))

    def test_destination_change_starts_a_new_count(self):
        b = Bench(3)
        for s in range(1, 4):
            b.tick(s)
            b.line(0, s + 0.1, Forward(1, 1, False))
            b.line(1, s + 0.2, DataRx(0, False))
        b.line(0, 4.0, DestChanged(2))
        for s in range(4, 6):
            b.tick(s)
            b.line(0, s + 0.1, Forward(2, 2, False))
            b.line(2, s + 0.2, DataRx(0, False))
        self.assertEqual(b.pair(0, 1).sent, 3)
        self.assertEqual((b.pair(0, 2).delivered, b.pair(0, 2).sent), (2, 2))

    def test_data_from_board_with_other_destination_ignored(self):
        b = Bench(3)
        b.line(0, 0.0, DestChanged(2))
        for s in range(1, 4):
            b.tick(s)
            b.line(1, s + 0.2, DataRx(0, False))        # left over from before the change
        self.assertIsNone(b.pair(0, 1))


class RelayTests(unittest.TestCase):
    def run_relay(self, relay_first):
        """N0 -> N2 -> N1 and N2 -> N1 itself; N2 prints a [FWD] for N0's packet and one for its own."""
        b = Bench(3)
        for s in range(1, 21):
            b.tick(s)
            up = (0, s + 0.10, Forward(1, 2, False))
            relay = (2, s + 0.11, Forward(1, 1, False))
            for rec in ([relay, up] if relay_first else [up, relay]):
                b.line(*rec)
            b.line(2, s + 0.5, Forward(1, 1, False))    # N2's own packet
            b.line(1, s + 0.2, DataRx(0, False))
            b.line(1, s + 0.6, DataRx(2, False))
        return b

    def test_relay_not_counted_as_sent(self):
        b = self.run_relay(relay_first=False)
        self.assertEqual((b.pair(2, 1).delivered, b.pair(2, 1).sent, b.pair(2, 1).relays), (20, 20, 20))
        self.assertEqual((b.pair(0, 1).delivered, b.pair(0, 1).sent), (20, 20))

    def test_relay_line_read_before_upstream_line(self):
        # Ports are read by separate threads, so N2's relay line can reach the dashboard first.
        b = self.run_relay(relay_first=True)
        self.assertLessEqual(abs(b.pair(2, 1).sent - 20), 1)

    def test_relay_for_unconnected_board_capped_by_time(self):
        # N3 is radio-only: N2 relays its packets but nothing shows "[FWD] .. via 2".
        b = Bench(3)
        for s in range(1, 31):
            b.tick(s)
            b.line(2, s + 0.1, Forward(1, 1, False))    # own
            b.line(2, s + 0.4, Forward(1, 1, False))    # relay for N3
            b.line(1, s + 0.2, DataRx(2, False))
            b.line(1, s + 0.5, DataRx(3, False))
        d = b.pair(2, 1)
        self.assertEqual(d.delivered, 30)
        self.assertLessEqual(d.sent, 30 + 2)          # at most SEND_BURST too many, not 60
        self.assertGreaterEqual(d.sent, 30)

    def test_jittery_read_times_not_mistaken_for_relays(self):
        b = Bench(2)
        t = 1.0
        for s in range(1, 41):
            b.tick(s)
            t += 0.9 if s % 2 else 1.1                  # PC read times wobble around the 1 s send interval
            b.line(0, t, Forward(1, 1, False))
        self.assertEqual(b.pair(0, 1).sent, 40)

    def test_upstream_match_expires(self):
        # N0 sent towards N2 but the packet never arrived: it must not cancel N2's sends much later.
        b = Bench(3)
        b.tick(1)
        b.line(0, 1.1, Forward(1, 2, False))
        for s in range(5, 10):
            b.tick(s)
            b.line(2, s + 0.1, Forward(1, 1, False))
        self.assertEqual(b.pair(2, 1).sent, 5)


class PanelTests(unittest.TestCase):
    def bench(self):
        b = Bench(3)
        for s in range(1, 11):
            b.tick(s)
            b.line(0, s + 0.1, Forward(1, 1, False))
            if s not in (4, 7):
                b.line(1, s + 0.2, DataRx(0, False))
        b.line(2, 10.5, DestChanged(5))                 # N5 is not plugged in
        return b

    def test_shown_per_sender(self):
        b = self.bench()
        text = draw(b.m, 10.5)
        self.assertIn("Delivery %", text)
        self.assertIn("N0 → N1", text)
        self.assertIn("8/10 (80%)", text)
        self.assertIn("N2 → N5", text)
        self.assertIn("dest not connected", text)
        self.assertNotIn("N1 → N1", text)               # N1's own destination is itself: sends nothing

    def test_paused_when_not_live(self):
        b = self.bench()
        self.assertIn("paused", draw(b.m, 30.0))

    def test_hide_delivery(self):
        text = draw(self.bench().m, 10.5, show_delivery=False)
        self.assertNotIn("Delivery", text)
        self.assertNotIn("8/10", text)

    def test_colours(self):
        self.assertEqual((DELIVERY_OK, DELIVERY_WARN), (85, 60))
        self.assertEqual(delivery_style(90), "green")
        self.assertEqual(delivery_style(70), "yellow")
        self.assertEqual(delivery_style(40), "bold red")

    def test_hide_delivery_flag(self):
        # Redirected output is only 80 columns wide, so check what main() passes to render().
        for extra, expect in (([], True), (["--hide-delivery"], False)):
            with mock.patch("mesh_dashboard.render", wraps=render) as spy, \
                    contextlib.redirect_stdout(io.StringIO()):
                main(["--replay", *SAMPLES, *extra])
            self.assertEqual(spy.call_args.kwargs["show_delivery"], expect)


if __name__ == "__main__":
    unittest.main()

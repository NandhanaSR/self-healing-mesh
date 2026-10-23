"""Tests for the event log: one row per event, repeats merged (×N), pause (space) and filter (f).

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import glob
import io
import os
import re
import unittest

from rich.console import Console

from keyboard import KeyControl
from live_model import MERGE_S, LiveMesh, replay, replay_now
from mesh_dashboard import render
from serial_parser import DestChanged, LinkEvent, NodeTable, Route

SAMPLES = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "samples", "mesh4", "node*.txt")))
T0 = 1_000_000.0


def draw(m, now, width=140, height=42, **kw):
    console = Console(file=io.StringIO(), width=width, height=height, record=True, color_system=None)
    console.print(render(m, now, **kw))
    return console.export_text().splitlines()


def event_rows(lines):
    """Text inside the EVENT LOG box (left column of the bottom row)."""
    top = next(i for i, l in enumerate(lines) if "EVENT LOG" in l)
    width = lines[top].index("┐")
    rows = []
    for l in lines[top + 1:]:
        if l.startswith("└"):
            return rows
        rows.append(l[1:width].strip())


def route_table(node, nxt, prio):
    return NodeTable(node, routes=(Route(1, nxt, 1.0, prio, 1),))


def link(old, new, q_pred=0.4):
    return LinkEvent(1, old, new, -80.0, -1.5, 0.6, 40, q_pred)


class LayoutTests(unittest.TestCase):
    def test_rows_at_minimum_size(self):
        # one row per event, plus a values row under each link change; nothing wraps
        m = replay(SAMPLES)
        rows = [r for r in event_rows(draw(m, replay_now(m), keys=KeyControl(m))) if r]
        self.assertEqual(len(rows), 10)
        for prev, r in zip([""] + rows, rows):
            if r.startswith("ewma "):
                self.assertIn("HEALTHY→DEGRADING", prev)
            else:
                self.assertRegex(r, r"^\d\d:\d\d:\d\d ")

    def test_link_values_in_full_at_minimum_size(self):
        m = replay(SAMPLES)
        rows = event_rows(draw(m, replay_now(m)))
        i = next(i for i, r in enumerate(rows) if "N3→N1  HEALTHY→DEGRADING  q_pred 0.37" in r)
        self.assertEqual(rows[i + 1], "ewma -83.1  slope -1.90  PRR 0.60  q 37%")

    def test_other_events_stay_one_row(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, route_table(2, 1, 1))
        m.apply("COM3", T0 + 1, route_table(2, 3, 1))
        rows = [r for r in event_rows(draw(m, T0 + 1)) if r]
        self.assertEqual(len(rows), 1)
        self.assertIn("next N1⇒N3  cost 1.00", rows[0])

    def test_footer_fits_at_minimum_size(self):
        m = LiveMesh(["COM11", "COM12", "COM14"])
        foot = draw(m, T0, keys=KeyControl(m))[-2]
        self.assertIn("f filter log · q quit", foot)


class ColourTests(unittest.TestCase):
    @staticmethod
    def styles(text):
        """{piece of text: its style} for a rich Text."""
        return {text.plain[s.start:s.end]: str(s.style) for s in text.spans}

    def test_link_rows_coloured_like_link_health(self):
        from mesh_dashboard import LINK_STYLE, link_event_text, link_values_text
        rec = LinkEvent(1, "HEALTHY", "DEGRADING", -83.1, -1.90, 0.60, 37, 0.37)
        first = self.styles(link_event_text(rec))
        self.assertEqual(first["HEALTHY"], LINK_STYLE["HEALTHY"])
        self.assertEqual(first["DEGRADING"], LINK_STYLE["DEGRADING"])
        self.assertEqual(first["0.37"], "bold red")             # q_pred below Q_TH
        values = link_values_text(rec)
        self.assertEqual(values.plain, " " * 9 + "ewma -83.1  slope -1.90  PRR 0.60  q 37%")
        values = self.styles(values)
        self.assertEqual(values["-1.90"], "red")                # falling, as in LINK HEALTH
        self.assertEqual(values["37%"], "bold red")
        self.assertEqual(values["-83.1"], "bright_white")
        self.assertEqual(values["ewma "], "dim")                # only the labels are dim

    def test_summary_is_bright(self):
        m = LiveMesh(["COM3"])
        m.log(T0, "INFO", "N0", "destination = N3")
        console = Console(file=io.StringIO(), width=140, height=42, record=True,
                          force_terminal=True, color_system="standard")
        console.print(render(m, T0))
        ansi = console.export_text(styles=True)
        self.assertIn("\x1b[97mdestination = N3", ansi)        # 97 = bright white, not dim (2)


class MergeTests(unittest.TestCase):
    def test_flapping_route_keeps_every_change(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, route_table(2, 1, 1))
        for i, nxt in enumerate([3, 1, 3], 1):
            m.apply("COM3", T0 + i, route_table(2, nxt, 1))
        self.assertEqual([(e.who, e.text, e.count) for e in m.events],
                         [("N2 →N1", "next N1⇒N3", 1), ("N2 →N1", "next N3⇒N1", 1), ("N2 →N1", "next N1⇒N3", 1)])

    def test_exact_repeat_is_one_row(self):
        m = LiveMesh(["COM3"])
        for i in range(3):
            m.apply("COM3", T0 + i, DestChanged(3))
        self.assertEqual([(e.msg, e.count) for e in m.events], [("COM3 destination = N3", 3)])
        self.assertIn("×3", "\n".join(draw(m, T0 + 3)))

    def test_different_value_is_a_new_row(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, DestChanged(3))
        m.apply("COM3", T0 + 1, DestChanged(4))
        m.apply("COM3", T0 + 2, DestChanged(3))           # N4 in between: not merged into the first row
        self.assertEqual([e.text for e in m.events], ["destination = N3", "destination = N4", "destination = N3"])

    def test_repeat_after_window_is_a_new_row(self):
        m = LiveMesh(["COM3"])
        m.log(T0, "INFO", "N0", "destination = N3")
        m.log(T0 + MERGE_S + 1, "INFO", "N0", "destination = N3")
        self.assertEqual([e.count for e in m.events], [1, 1])

    def test_merged_row_moves_to_top(self):
        m = LiveMesh(["COM3", "COM4"])
        m.apply("COM3", T0, DestChanged(3))
        m.apply("COM4", T0 + 1, DestChanged(2))
        m.apply("COM3", T0 + 2, DestChanged(3))           # another board's event in between is fine
        self.assertEqual([(e.msg, e.count) for e in m.events],
                         [("COM4 destination = N2", 1), ("COM3 destination = N3", 2)])

    def test_link_is_one_way_as_measured(self):
        m = LiveMesh(["COM3", "COM4"])
        m.apply("COM3", T0, NodeTable(2))
        m.apply("COM4", T0, NodeTable(1))
        m.apply("COM3", T0 + 1, link("HEALTHY", "DEGRADING"))                        # N2 measures N1
        m.apply("COM4", T0 + 1, LinkEvent(2, "HEALTHY", "DEGRADING", -80.0, -1.5, 0.6, 40, 0.4))  # N1 measures N2
        self.assertEqual([e.who for e in m.events], ["N2→N1", "N1→N2"])
        self.assertIn("N2→N1", "\n".join(draw(m, T0 + 1)))

    def test_link_back_and_forth_stays_separate(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, NodeTable(2))
        for i, (old, new) in enumerate([("HEALTHY", "DEGRADING"), ("DEGRADING", "HEALTHY"),
                                        ("HEALTHY", "DEGRADING")], 1):
            m.apply("COM3", T0 + i, link(old, new))
        self.assertEqual([(e.icon, e.text.split()[0], e.count) for e in m.events],
                         [("▼", "HEALTHY→DEGRADING", 1), ("▲", "DEGRADING→HEALTHY", 1), ("▼", "HEALTHY→DEGRADING", 1)])

    def test_same_link_transition_repeated_merges(self):
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, NodeTable(2))
        m.apply("COM3", T0 + 1, link("HEALTHY", "DEGRADING", 0.45))
        m.apply("COM3", T0 + 2, link("HEALTHY", "DEGRADING", 0.42))
        self.assertEqual([(e.text, e.count) for e in m.events], [("HEALTHY→DEGRADING  q_pred 0.42", 2)])

    def test_link_transitions_each_keep_a_row(self):
        # HEALTHY -> DEGRADING before DEAD is the prediction working; never fold those together
        m = LiveMesh(["COM3"])
        m.apply("COM3", T0, NodeTable(2))
        m.apply("COM3", T0 + 1, link("HEALTHY", "DEGRADING"))
        m.apply("COM3", T0 + 2, link("DEGRADING", "DEAD", 0.0))
        self.assertEqual([(e.icon, e.text) for e in m.events],
                         [("▼", "HEALTHY→DEGRADING  q_pred 0.40"), ("✗", "DEGRADING→DEAD  q_pred 0.00")])


class PauseFilterTests(unittest.TestCase):
    def setUp(self):
        self.m = LiveMesh(["COM3"])
        self.k = KeyControl(self.m)
        self.m.log(T0, "INFO", "COM3", "open")
        self.m.log(T0 + 1, "WARN", "N2→N1", "HEALTHY→DEGRADING")

    def text(self):
        return "\n".join(draw(self.m, T0 + 5, keys=self.k))

    def test_pause_freezes_rows_and_counts_new(self):
        self.k.handle(" ", T0)
        self.m.log(T0 + 2, "CRIT", "N2→N1", "DEGRADING→DEAD")
        self.m.log(T0 + 3, "ROUTE", "N2 →N1", "next N1⇒N3")
        text = self.text()
        self.assertIn("PAUSED · 2 new", text)
        self.assertNotIn("DEGRADING→DEAD", text)
        self.assertEqual(len(self.m.events), 4)             # still recorded while paused
        self.k.handle(" ", T0)
        text = self.text()
        self.assertNotIn("PAUSED", text)
        self.assertIn("DEGRADING→DEAD", text)

    def test_filter_hides_info(self):
        self.assertIn("• COM3", self.text())
        self.k.handle("f", T0)
        text = self.text()
        self.assertIn("newest first · important", text)
        self.assertNotIn("• COM3", text)
        self.assertIn("HEALTHY→DEGRADING", text)
        self.k.handle("f", T0)
        self.assertIn("newest first · all", self.text())

    def test_pause_and_filter_send_nothing(self):
        sent = []
        self.m.send = lambda port, key: sent.append(key) or True
        self.k.handle(" ", T0)
        self.k.handle("f", T0)
        self.assertEqual(sent, [])


if __name__ == "__main__":
    unittest.main()

"""Render tests for mesh_dashboard.py: draw frames off-screen from sample captures (no boards needed).

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import glob
import io
import os
import unittest

from rich.console import Console

from live_model import LiveMesh, PortStatus, replay, replay_now
from mesh_dashboard import render

SAMPLES = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "samples", "mesh4", "node*.txt")))


def draw(mesh, now, **kw):
    console = Console(file=io.StringIO(), width=170, height=50, record=True, color_system=None)
    console.print(render(mesh, now, **kw))
    return console.export_text()


class RenderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mesh = replay(SAMPLES)
        cls.text = draw(cls.mesh, replay_now(cls.mesh), mode="REPLAY")

    def test_samples_found(self):
        self.assertEqual(len(SAMPLES), 4)

    def test_firmware_terms_and_prediction_columns(self):
        for word in ("SLOPE", "dPRR", "Q_PRED", "DEGRADING", "HEALTHY", "PRIO", "REPLAY"):
            self.assertIn(word, self.text)

    def test_no_simulator_leftovers(self):
        for word in ("LoRa", "MAC", "GOOD", "CRITICAL", "GATEWAY", "handover", "latency", "SIM"):
            self.assertNotIn(word, self.text)

    def test_link_values_shown_as_printed(self):
        self.assertIn("N3→N1", self.text)
        self.assertIn("-83.1", self.text)       # node3 nbr 1 ewma
        self.assertIn("-1.90", self.text)       # its slope
        self.assertIn("0.37", self.text)        # its q_pred

    def test_rerouted_path_follows_next_hops(self):
        # N3's route to N1 moved off the degrading link: N3 -> N2 -> N1.
        self.assertIn("N3 ─0.87→ N2 ─0.94→ N1", self.text)

    def test_destination_marker_and_unknown_dest(self):
        self.assertIn("★N1", self.text)           # node0 printed "destination = 1"
        self.assertIn("N2→N1?", self.text)        # node2 never printed its destination

    def test_counts(self):
        b1 = self.mesh.by_id()[1]
        self.assertEqual((b1.delivered, dict(b1.delivered_from)), (4, {0: 2, 2: 1, 3: 1}))
        self.assertEqual(self.mesh.link_changes, 2)
        self.assertEqual(self.mesh.route_changes, 2)  # N1->N3 priority next, N3->N1 next


class DemoLabelTests(unittest.TestCase):
    def test_anomaly_panel_is_labelled_demo(self):
        from demo_anomaly import DemoAnomalies
        m = replay(SAMPLES)
        a = DemoAnomalies(seed=1)
        for _ in range(10):
            a.step(m.by_id())
        text = draw(m, replay_now(m), mode="REPLAY", anomalies=a)
        self.assertIn("ANOMALY  DEMO", text)
        self.assertIn("generated · no sensors", text)
        self.assertIn("N3 s1", text)

    def test_demo_label_even_live(self):
        self.assertIn("DEMO", draw(LiveMesh(["COM3"]), 0.0))

    def test_sim_banner(self):
        from mesh_dashboard import SIM_MODE
        _, mesh, _ = __import__("test_demo").run_sim(5)
        text = draw(mesh, max(b.table_time for b in mesh.boards.values()), mode=SIM_MODE)
        self.assertIn("SIM (simulated boards, not real)", text)
        self.assertNotIn("● LIVE", text)


class EmptyStateTests(unittest.TestCase):
    def test_nothing_connected(self):
        m = LiveMesh(["COM3", "COM4"])
        m.apply("COM3", 0.0, PortStatus(False, "could not open port 'COM3'"))
        text = draw(m, 10.0)
        self.assertIn("LIVE", text)
        self.assertIn("OFFLN", text)
        self.assertIn("0/2 boards", text)
        self.assertIn("no link data yet", text)

    def test_stale_board(self):
        m = replay(SAMPLES[:1])
        m.apply(SAMPLES[0], 0.0, PortStatus(True))
        self.assertIn("STALE", draw(m, replay_now(m) + 10.0))


if __name__ == "__main__":
    unittest.main()

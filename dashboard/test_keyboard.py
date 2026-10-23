"""Tests for keyboard.py: which board gets each key, and how raw terminal input is split.

Run from the project root:   python -m unittest discover -s dashboard -v
"""
import io
import time
import unittest

from rich.console import Console

from demo_sim import SimMesh
from keyboard import KeyControl, Keyboard, split_posix_keys
from live_model import LiveMesh
from mesh_dashboard import REPLAY_MODE, render


class FakeMesh(LiveMesh):
    """A LiveMesh whose send() records writes instead of using serial ports."""

    def __init__(self, ports, open_ports=None):
        super().__init__(ports)
        self.open_ports = set(ports if open_ports is None else open_ports)
        self.sent = []

    def send(self, port, text):
        if port not in self.open_ports:
            return False
        self.sent.append((port, text))
        return True


class KeyControlTests(unittest.TestCase):
    def test_keys_go_to_first_port_by_default(self):
        m = FakeMesh(["COM3", "COM4"])
        k = KeyControl(m)
        for key in "3p":
            self.assertTrue(k.handle(key, 0.0))
        self.assertEqual(m.sent, [("COM3", "3"), ("COM3", "p")])

    def test_selection_cycles_in_ports_order(self):
        m = FakeMesh(["COM5", "COM3", "COM4"])      # --ports order, not sorted
        k = KeyControl(m)
        seen = []
        for key in ["\t", "]", "\t", "["]:
            k.handle(key, 0.0)
            seen.append(k.selected)
        self.assertEqual(seen, ["COM3", "COM4", "COM5", "COM4"])
        k.handle("0", 0.0)
        self.assertEqual(m.sent, [("COM4", "0")])

    def test_every_firmware_key_is_sent(self):
        m = FakeMesh(["COM3"])
        k = KeyControl(m)
        for key in "012345p":
            k.handle(key, 0.0)
        self.assertEqual([t for _, t in m.sent], list("012345p"))

    def test_other_keys_send_nothing(self):
        m = FakeMesh(["COM3"])
        k = KeyControl(m)
        for key in "6789xP\r\n\x1b":
            self.assertTrue(k.handle(key, 0.0))
        self.assertEqual(m.sent, [])
        self.assertEqual(list(m.events), [])

    def test_quit(self):
        k = KeyControl(FakeMesh(["COM3"]))
        self.assertFalse(k.handle("q", 0.0))
        self.assertFalse(k.handle("\x03", 0.0))

    def test_send_logged_with_node_id_once_known(self):
        m = FakeMesh(["COM3"])
        k = KeyControl(m)
        k.handle("2", 0.0)
        m.boards["COM3"].node_id = 4
        k.handle("p", 0.0)
        msgs = [e.msg for e in m.events]
        self.assertEqual(msgs, ["COM3 sent '2'", "N4 (COM3) sent 'p'"])

    def test_port_not_open(self):
        m = FakeMesh(["COM3", "COM4"], open_ports=["COM3"])
        k = KeyControl(m)
        k.handle("]", 0.0)
        k.handle("1", 0.0)
        self.assertEqual(m.sent, [])
        self.assertEqual((m.events[-1].level, m.events[-1].msg), ("WARN", "COM4 not open, key '1' ignored"))

    def test_no_ports(self):
        k = KeyControl(FakeMesh([]))
        self.assertTrue(k.handle("3", 0.0))
        self.assertTrue(k.handle("\t", 0.0))
        self.assertIsNone(k.selected)


class PosixSplitTests(unittest.TestCase):
    def test_plain_keys(self):
        self.assertEqual(split_posix_keys("3p\tq"), ["3", "p", "\t", "q"])

    def test_escape_sequences_dropped(self):
        # up arrow, F5, then a real key; a lone ESC is dropped too
        self.assertEqual(split_posix_keys("\x1b[A\x1b[15~2\x1bOP\x1b"), ["2"])


class KeyboardTests(unittest.TestCase):
    def test_disabled_when_not_a_terminal(self):
        with Keyboard(io.StringIO("3p")) as kb:
            self.assertFalse(kb.enabled)
            self.assertEqual(kb.read(), [])


class RenderTests(unittest.TestCase):
    def draw(self, m, **kw):
        console = Console(file=io.StringIO(), width=170, height=50, record=True, color_system=None)
        console.print(render(m, 0.0, **kw))
        return console.export_text()

    def test_selected_board_marked_and_footer(self):
        m = FakeMesh(["COM3", "COM4"])
        m.boards["COM4"].node_id = 2
        k = KeyControl(m)
        k.handle("]", 0.0)
        text = self.draw(m, keys=k)
        self.assertIn("▶N2", text)
        self.assertIn("keys →  N2 (COM4)", text)
        self.assertIn("Tab/[ ] board · 0-5 dest · p prio · space pause log · f filter log · q quit", text)

    def test_footer_without_keyboard(self):
        self.assertIn("keyboard off", self.draw(FakeMesh(["COM3"])))
        self.assertIn("no keys", self.draw(FakeMesh(["COM3"]), mode=REPLAY_MODE))


class DemoKeysTests(unittest.TestCase):
    def test_key_reaches_simulated_board(self):
        """--demo path: key -> KeyControl -> SerialReader -> SimMesh, reply parsed back into the model."""
        sim = SimMesh(seed=2)
        mesh = LiveMesh(sim.ports)
        mesh.start(opener=sim.open)
        try:
            k = KeyControl(mesh)
            k.handle("]", 0.0)                       # SIM1
            deadline = time.time() + 5
            while not mesh.send(k.selected, "") and time.time() < deadline:
                time.sleep(0.05)                     # wait for the reader to open the port
            k.handle("4", 0.0)
            k.handle("p", 0.0)
            board = mesh.boards[k.selected]
            while (board.dest != 4 or board.priority is not True) and time.time() < deadline:
                time.sleep(0.05)
                mesh.poll()
            self.assertEqual(board.dest, 4)
            self.assertTrue(board.priority)
        finally:
            mesh.stop()


if __name__ == "__main__":
    unittest.main()

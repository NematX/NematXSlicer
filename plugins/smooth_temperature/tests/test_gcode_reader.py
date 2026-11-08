"""Fast checks of the package's G-code oracle, 3MF copies and timeout launcher.

Short commands exercise state changes and deliberately incomplete outputs. A
temporary process tree proves that a budget failure is visible and cleans up its
descendant; no slicer or host DLL is needed.

    unittest -> parse_gcode / write_project / HostRunner.run
"""
import ctypes
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET
from zipfile import ZipFile

from gcode_reader import parse_gcode
from host_runner import HostRunner
from test_smooth_temperature import Object, write_project


class Reader(unittest.TestCase):
    def test_absolute_relative_resets_retractions_and_arcs(self):
        """Ignore retractions/pure E moves and follow both extrusion coordinate modes."""
        result = parse_gcode([
            "M104 S200", ";LAYER_CHANGE", "; printing object a id:0 copy 0", "M82", "G92 E10",
            "G1 X1 E9", "G1 E10", "G1 X2 E11", "M104 S210", "G92 E0", "G2 X3 E1",
            "M83", "M104 S220", "G3 Y1 E0.5", "G1 X4 E-1", "M104 S230", "G1 E1",
        ])
        self.assertEqual(result.series("a", 1), [[200, 210, 220]])

    def test_tools_waits_and_explicit_heater(self):
        """T in a temperature request does not switch the active extrusion tool."""
        result = parse_gcode(["M104 S200 T0", "M109 R215 T1", "M83", ";LAYER_CHANGE",
                              "; printing object a id:0 copy 0", "G1 X1 E1", "T1", "G1 X2 E1",
                              "M109 S225", "G1 X3 E1"])
        self.assertEqual(result.series("a", 1, tool=0), [[200]])
        self.assertEqual(result.series("a", 1, tool=1), [[215, 225]])
        self.assertEqual(result.waits, [(-1, 1, 215), (0, 1, 225)])

    def test_copies_and_identical_names_are_not_merged(self):
        """Copy ids stay distinct; two objects with the same name are ambiguous."""
        result = parse_gcode(["M83", "M104 S200", ";LAYER_CHANGE", "; printing object same id:0 copy 0",
                              "G1 X1 E1", "; printing object same id:0 copy 1", "M104 S230", "G1 X2 E1"])
        self.assertEqual(result.series("same", 1, copy=0), [[200]])
        self.assertEqual(result.series("same", 1, copy=1), [[230]])
        with self.assertRaises(AssertionError):
            parse_gcode(result.gcode + ["; printing object same id:1 copy 0", "G1 X3 E1"]).series("same", 2)

    def test_missing_setpoints_layers_and_malformed_labels_fail(self):
        """A partial export or unknown heater state must not become a passing subset."""
        with self.assertRaisesRegex(AssertionError, "known setpoint"):
            parse_gcode(["M83", ";LAYER_CHANGE", "; printing object a id:0 copy 0", "G1 X1 E1"])
        with self.assertRaisesRegex(AssertionError, "expected 2 layers"):
            parse_gcode(["M104 S200", "M83", ";LAYER_CHANGE", "; printing object a id:0 copy 0", "G1 X1 E1"]).series("a", 2)
        with self.assertRaisesRegex(AssertionError, "Unreadable object"):
            parse_gcode(["; printing object a"])
        with self.assertRaises(AssertionError):
            parse_gcode([]).series("a", 1)
        with self.assertRaisesRegex(AssertionError, "wrong printed layer positions"):
            parse_gcode(["M104 S200", "M83", ";LAYER_CHANGE", "; printing object a id:0 copy 0", "G1 X1 E1",
                         ";LAYER_CHANGE", ";LAYER_CHANGE", "G1 X2 E1"]).series("a", 2)

    def test_synthetic_paths_and_repeated_values(self):
        """Keep regionless extrusions separate and retain a return to an earlier temperature."""
        result = parse_gcode(["M83", "M104 S200", ";LAYER_CHANGE", ";TYPE:Skirt", "G1 X1 E1",
                              "; printing object a id:0 copy 0", "G1 X2 E1", "M104 S250", "G1 X3 E1",
                              "M104 S200", "G1 X4 E1", "; stop printing object a id:0 copy 0"])
        self.assertEqual(result.synthetic, [(0, "Skirt", 200)])
        self.assertEqual(result.series("a", 1), [[200, 250, 200]])
        self.assertEqual(result.synthetic_heaters, [])

    def test_3mf_copies_share_one_mesh(self):
        """The integration fixture really makes two instances, not two duplicated meshes."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "copies.3mf"
            write_project(path, [Object("same", copies=(0, 80))])
            with ZipFile(path) as archive:
                root = ET.fromstring(archive.read("3D/3dmodel.model"))
            ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
            self.assertEqual(root.find("m:metadata", ns).text, "1", "keep instances grouped when the host imports the project")
            self.assertEqual(len(root.findall("m:resources/m:object", ns)), 1)
            items = root.findall("m:build/m:item", ns)
            self.assertEqual([item.attrib["objectid"] for item in items], ["1", "1"])
            self.assertNotEqual(items[0].attrib["transform"], items[1].attrib["transform"])


class Deadline(unittest.TestCase):
    def test_expired_budget_starts_no_command(self):
        """The shared deadline is checked before a new process can be launched."""
        runner = HostRunner(-1, dict(os.environ))
        with self.assertRaisesRegex(AssertionError, "global wall-clock budget"):
            runner.run(["must-not-exist"], Path("must-not-be-written.log"))
        self.assertEqual(runner.calls, [])

    def test_failure_keeps_log_and_exit_status(self):
        """Do not turn a failed child into a successful test or discard its output."""
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "host.log"
            runner = HostRunner(2, dict(os.environ))
            with self.assertRaisesRegex(AssertionError, "exited with 7"):
                runner.run([sys.executable, "-c", "print('diagnostic', flush=True); raise SystemExit(7)"], log)
            self.assertIn("diagnostic", log.read_text())
            self.assertEqual(runner.calls[0]["exit"], 7)

    def test_timeout_stops_descendant_and_keeps_partial_log(self):
        """A timed-out host cannot leave a child running after the suite returns."""
        with tempfile.TemporaryDirectory() as directory:
            log, pidfile = Path(directory) / "host.log", Path(directory) / "child.pid"
            child = "import time; time.sleep(30)"
            parent = ("import pathlib,subprocess,sys,time; "
                      f"p=subprocess.Popen([sys.executable,'-c',{child!r}]); "
                      f"pathlib.Path({str(pidfile)!r}).write_text(str(p.pid)); "
                      "print('started',flush=True); time.sleep(30)")
            runner = HostRunner(2, dict(os.environ))
            start = time.monotonic()
            with self.assertRaises(subprocess.TimeoutExpired):
                runner.run([sys.executable, "-c", parent], log, per_command=0.3)
            self.assertLess(time.monotonic() - start, 2)
            self.assertIn("started", log.read_text())
            pid = int(pidfile.read_text())
            if os.name == "nt":
                kernel = ctypes.WinDLL("kernel32", use_last_error=True)
                kernel.OpenProcess.restype = ctypes.c_void_p
                handle = kernel.OpenProcess(0x1000, False, pid)
                if handle:
                    code = ctypes.c_ulong()
                    try:
                        self.assertTrue(kernel.GetExitCodeProcess(ctypes.c_void_p(handle), ctypes.byref(code)))
                        self.assertNotEqual(code.value, 259, "descendant is still running")
                    finally:
                        kernel.CloseHandle(ctypes.c_void_p(handle))
            else:
                # A killed child may briefly remain as a zombie until init reaps it.
                status = Path(f"/proc/{pid}/stat")
                if status.exists():
                    self.assertEqual(status.read_text().split()[2], "Z")


if __name__ == "__main__":
    unittest.main(verbosity=2)

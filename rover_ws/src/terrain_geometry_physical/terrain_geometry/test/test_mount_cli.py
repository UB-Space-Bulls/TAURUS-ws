"""Regression coverage for blank fields that previously produced broken YAML."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'create_mount'


class MountCliTests(unittest.TestCase):
    def test_invalid_input_is_reprompted_and_written_as_numbers(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'mount.yaml'
            run = subprocess.run([sys.executable, str(SCRIPT), '--output', str(output)],
                input='\nnan\n0.8\n3.141592653589793\n\n0\n0\n',
                text=True, capture_output=True, timeout=5)
            self.assertEqual(run.returncode, 0, run.stderr)
            data = yaml.safe_load(output.read_text())
            self.assertEqual(data['z'], 0.8)
            self.assertEqual(data['pitch'], 0.0)
            self.assertEqual(data['yaw'], 0.0)
            self.assertEqual(data['parent_frame'], 'terrain_map')

    def test_existing_mount_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'mount.yaml'
            output.write_text('original measurements\n')
            run = subprocess.run([sys.executable, str(SCRIPT), '--output', str(output)],
                                 input='', text=True, capture_output=True, timeout=5)
            self.assertNotEqual(run.returncode, 0)
            self.assertEqual(output.read_text(), 'original measurements\n')

    def test_cancel_does_not_leave_partial_mount(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'mount.yaml'
            run = subprocess.run([sys.executable, str(SCRIPT), '--output', str(output)],
                                 input='0.8\n0\n', text=True, capture_output=True, timeout=5)
            self.assertNotEqual(run.returncode, 0)
            self.assertFalse(output.exists())

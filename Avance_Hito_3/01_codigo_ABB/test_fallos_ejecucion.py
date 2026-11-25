from pathlib import Path
import re
import unittest
from fallos_ejecucion import ProgressWatchdog, failure_message


class FailureHandlingTests(unittest.TestCase):
    def test_stall_is_detected_without_retries_or_commands(self):
        watch = ProgressWatchdog(timeout_s=12.)
        self.assertFalse(watch.observe(0., 4, 3, 10.))
        self.assertFalse(watch.observe(11.9, 4, 3, 10.))
        self.assertTrue(watch.observe(12., 4, 3, 10.))

    def test_small_cumulative_progress_and_new_point_reset_watch(self):
        watch = ProgressWatchdog(timeout_s=5.)
        for t in range(30):
            self.assertFalse(watch.observe(float(t), 4, 3, 10.-t*.0006))
        self.assertFalse(watch.observe(33., 5, 4, 20.))
        self.assertFalse(watch.observe(37., 5, 4, 20.))
        self.assertTrue(watch.observe(38., 5, 4, 20.))

    def test_clear_rejection_reports_target_axis_and_magnitude(self):
        text = failure_message(-3, point=4, axis=5, delta=.073)
        self.assertIn('P4', text)
        self.assertIn('J5', text)
        self.assertIn('0.073000', text)

    def test_rapid_never_uses_program_exit_as_loop_break(self):
        source = Path(__file__).with_name('rapid').joinpath('IMT342_Automatico.mod').read_text(encoding='ascii')
        self.assertIsNone(re.search(r'(?im)^\s*EXIT\s*;', source))
        self.assertIn('WaitRob \\InPos;', source)
        self.assertIn('GuardarDiferencia actual.robax,objetivo.robax;', source)

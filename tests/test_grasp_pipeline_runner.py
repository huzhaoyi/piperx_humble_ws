import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "grasp_task_manager"),
)

from grasp_task_manager.pipeline_runner import (
    GraspPipelineConfig,
    GraspPipelineRunner,
)


class RecordingCommandRunner:
    def __init__(self):
        self.commands = []

    def run(self, command, timeout=None):
        self.commands.append((list(command), timeout))
        return 0


class GraspPipelineRunnerTest(unittest.TestCase):
    def test_plan_then_execute_reuses_frozen_offset(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            frozen_path = Path(tmpdir) / "selected.json"
            report_path = Path(tmpdir) / "report.json"
            execute_report_path = Path(tmpdir) / "execute.json"
            frozen_path.write_text(
                json.dumps({"tcp_grasp_offset_z": 0.10}),
                encoding="utf-8",
            )
            command_runner = RecordingCommandRunner()
            runner = GraspPipelineRunner(
                GraspPipelineConfig(
                    frozen_selection_path=str(frozen_path),
                    plan_report_path=str(report_path),
                    execute_report_path=str(execute_report_path),
                    max_candidates=12,
                    execute=True,
                ),
                command_runner=command_runner,
            )

            runner.run()

            commands = [item[0] for item in command_runner.commands]
            self.assertEqual(commands[0][:2], ["python3", "scripts/piperx_grasp_preflight.py"])
            self.assertEqual(commands[1][:2], ["python3", "scripts/anygrasp_apply_scene.py"])
            self.assertIn("--save-selected-path", commands[2])
            self.assertIn(str(frozen_path), commands[2])
            self.assertIn("--max-candidates", commands[2])
            self.assertIn("12", commands[2])
            self.assertIn("--frozen-selection-path", commands[3])
            self.assertIn(str(frozen_path), commands[3])
            self.assertIn("--tcp-grasp-offset-z", commands[3])
            self.assertIn("0.100", commands[3])
            self.assertIn("--execute", commands[3])

    def test_plan_only_skips_execute_stage(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            command_runner = RecordingCommandRunner()
            runner = GraspPipelineRunner(
                GraspPipelineConfig(
                    frozen_selection_path=str(Path(tmpdir) / "selected.json"),
                    execute=False,
                ),
                command_runner=command_runner,
            )

            runner.run()

            commands = [item[0] for item in command_runner.commands]
            self.assertEqual(len(commands), 3)
            self.assertFalse(any("--execute" in command for command in commands))

    def test_move_home_adds_preflight_move_home_flag(self):
        command_runner = RecordingCommandRunner()
        runner = GraspPipelineRunner(
            GraspPipelineConfig(move_home=True),
            command_runner=command_runner,
        )

        runner.run()

        first_command = command_runner.commands[0][0]
        self.assertEqual(first_command[:2], ["python3", "scripts/piperx_grasp_preflight.py"])
        self.assertIn("--move-home", first_command)


if __name__ == "__main__":
    unittest.main()

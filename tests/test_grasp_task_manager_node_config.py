import sys
import unittest
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "grasp_task_manager"),
)

from grasp_task_manager.pipeline_runner import build_pipeline_config


class GraspTaskManagerNodeConfigTest(unittest.TestCase):
    def test_build_pipeline_config_for_execute_action(self):
        config = build_pipeline_config(
            {
                "frozen_selection_path": "tmp/selected.json",
                "plan_report_path": "tmp/plan.json",
                "execute_report_path": "tmp/execute.json",
                "tcp_grasp_offset_candidates": "0.12,0.10",
                "max_candidates": 7,
                "move_home_before_execute": True,
            },
            execute=True,
        )

        self.assertEqual(config.frozen_selection_path, "tmp/selected.json")
        self.assertEqual(config.plan_report_path, "tmp/plan.json")
        self.assertEqual(config.execute_report_path, "tmp/execute.json")
        self.assertEqual(config.tcp_grasp_offset_candidates, "0.12,0.10")
        self.assertEqual(config.max_candidates, 7)
        self.assertTrue(config.move_home)
        self.assertTrue(config.execute)


if __name__ == "__main__":
    unittest.main()

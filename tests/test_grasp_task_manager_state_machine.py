import unittest
import sys
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "grasp_task_manager"),
)

from grasp_task_manager.state_machine import (
    GraspEvent,
    GraspState,
    GraspTaskStateMachine,
)


class GraspTaskStateMachineTest(unittest.TestCase):
    def test_starts_idle_and_locks_after_plan_success(self):
        machine = GraspTaskStateMachine()

        self.assertEqual(machine.state, GraspState.IDLE)

        machine.handle(GraspEvent.START_REQUESTED)
        machine.handle(GraspEvent.CANDIDATE_LOCKED)
        machine.handle(GraspEvent.PLAN_SUCCEEDED)

        self.assertEqual(machine.state, GraspState.LOCKED)
        self.assertTrue(machine.target_locked)
        self.assertFalse(machine.candidate_updates_allowed)

    def test_execution_disables_candidate_updates(self):
        machine = GraspTaskStateMachine()
        machine.handle(GraspEvent.START_REQUESTED)
        machine.handle(GraspEvent.CANDIDATE_LOCKED)
        machine.handle(GraspEvent.PLAN_SUCCEEDED)

        machine.handle(GraspEvent.EXECUTE_REQUESTED)

        self.assertEqual(machine.state, GraspState.EXECUTING_PREGRASP)
        self.assertFalse(machine.candidate_updates_allowed)

    def test_empty_grasp_enters_recovery(self):
        machine = GraspTaskStateMachine()
        for event in (
            GraspEvent.START_REQUESTED,
            GraspEvent.CANDIDATE_LOCKED,
            GraspEvent.PLAN_SUCCEEDED,
            GraspEvent.EXECUTE_REQUESTED,
            GraspEvent.PREGRASP_REACHED,
            GraspEvent.APPROACH_COMPLETE,
            GraspEvent.GRIPPER_CLOSED,
            GraspEvent.EMPTY_GRASP_DETECTED,
        ):
            machine.handle(event)

        self.assertEqual(machine.state, GraspState.RECOVERING)
        self.assertFalse(machine.target_locked)

    def test_abort_from_any_state_goes_to_aborted(self):
        machine = GraspTaskStateMachine()
        machine.handle(GraspEvent.START_REQUESTED)

        machine.handle(GraspEvent.ABORT_REQUESTED)

        self.assertEqual(machine.state, GraspState.ABORTED)
        self.assertFalse(machine.target_locked)

    def test_can_require_home_before_detection(self):
        machine = GraspTaskStateMachine(require_home_before_detection=True)

        machine.handle(GraspEvent.START_REQUESTED)

        self.assertEqual(machine.state, GraspState.HOMING)
        self.assertFalse(machine.candidate_updates_allowed)

        machine.handle(GraspEvent.HOME_REACHED)
        machine.handle(GraspEvent.START_REQUESTED)

        self.assertEqual(machine.state, GraspState.DETECTING)

    def test_home_failure_enters_recovery(self):
        machine = GraspTaskStateMachine(require_home_before_detection=True)
        machine.handle(GraspEvent.START_REQUESTED)

        machine.handle(GraspEvent.HOME_FAILED)

        self.assertEqual(machine.state, GraspState.RECOVERING)


if __name__ == "__main__":
    unittest.main()

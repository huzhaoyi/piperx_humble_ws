from dataclasses import dataclass
from enum import Enum, auto


class GraspState(Enum):
    IDLE = auto()
    HOMING = auto()
    READY = auto()
    DETECTING = auto()
    LOCKED = auto()
    EXECUTING_PREGRASP = auto()
    APPROACHING = auto()
    CLOSING = auto()
    VERIFYING_GRASP = auto()
    LIFTING = auto()
    COMPLETED = auto()
    RECOVERING = auto()
    ABORTED = auto()
    FAULT = auto()


class GraspEvent(Enum):
    START_REQUESTED = auto()
    HOME_REQUESTED = auto()
    HOME_REACHED = auto()
    HOME_FAILED = auto()
    CANDIDATE_LOCKED = auto()
    PLAN_SUCCEEDED = auto()
    PLAN_FAILED = auto()
    EXECUTE_REQUESTED = auto()
    PREGRASP_REACHED = auto()
    APPROACH_COMPLETE = auto()
    GRIPPER_CLOSED = auto()
    GRASP_VERIFIED = auto()
    EMPTY_GRASP_DETECTED = auto()
    LIFT_COMPLETE = auto()
    ABORT_REQUESTED = auto()
    FAULT_DETECTED = auto()
    RESET_REQUESTED = auto()


@dataclass
class GraspTaskStateMachine:
    state: GraspState = GraspState.IDLE
    target_locked: bool = False
    require_home_before_detection: bool = False

    @property
    def candidate_updates_allowed(self) -> bool:
        return self.state in {
            GraspState.IDLE,
            GraspState.READY,
            GraspState.DETECTING,
        } and not self.target_locked

    def handle(self, event: GraspEvent) -> GraspState:
        if event == GraspEvent.ABORT_REQUESTED:
            self.target_locked = False
            self.state = GraspState.ABORTED
            return self.state
        if event == GraspEvent.FAULT_DETECTED:
            self.target_locked = False
            self.state = GraspState.FAULT
            return self.state
        if event == GraspEvent.RESET_REQUESTED:
            self.target_locked = False
            self.state = GraspState.IDLE
            return self.state

        if self.state == GraspState.IDLE and event == GraspEvent.START_REQUESTED:
            if self.require_home_before_detection:
                self.state = GraspState.HOMING
            else:
                self.state = GraspState.DETECTING
            return self.state

        transitions = {
            (GraspState.IDLE, GraspEvent.HOME_REQUESTED): GraspState.HOMING,
            (GraspState.HOMING, GraspEvent.HOME_REACHED): GraspState.READY,
            (GraspState.HOMING, GraspEvent.HOME_FAILED): GraspState.RECOVERING,
            (GraspState.READY, GraspEvent.START_REQUESTED): GraspState.DETECTING,
            (GraspState.DETECTING, GraspEvent.CANDIDATE_LOCKED): GraspState.DETECTING,
            (GraspState.DETECTING, GraspEvent.PLAN_SUCCEEDED): GraspState.LOCKED,
            (GraspState.DETECTING, GraspEvent.PLAN_FAILED): GraspState.RECOVERING,
            (GraspState.LOCKED, GraspEvent.EXECUTE_REQUESTED): GraspState.EXECUTING_PREGRASP,
            (GraspState.EXECUTING_PREGRASP, GraspEvent.PREGRASP_REACHED): GraspState.APPROACHING,
            (GraspState.APPROACHING, GraspEvent.APPROACH_COMPLETE): GraspState.CLOSING,
            (GraspState.CLOSING, GraspEvent.GRIPPER_CLOSED): GraspState.VERIFYING_GRASP,
            (GraspState.VERIFYING_GRASP, GraspEvent.GRASP_VERIFIED): GraspState.LIFTING,
            (GraspState.VERIFYING_GRASP, GraspEvent.EMPTY_GRASP_DETECTED): GraspState.RECOVERING,
            (GraspState.LIFTING, GraspEvent.LIFT_COMPLETE): GraspState.COMPLETED,
        }

        key = (self.state, event)
        if key not in transitions:
            return self.state

        if event == GraspEvent.CANDIDATE_LOCKED:
            self.target_locked = True
        if event == GraspEvent.PLAN_SUCCEEDED:
            self.target_locked = True
        if event in {GraspEvent.PLAN_FAILED, GraspEvent.EMPTY_GRASP_DETECTED}:
            self.target_locked = False

        self.state = transitions[key]
        return self.state

import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional


class PipelineStageError(RuntimeError):
    def __init__(self, stage: str, message: str):
        super().__init__(f"{stage}: {message}")
        self.stage = stage


class SubprocessCommandRunner:
    def run(self, command, timeout=None):
        completed = subprocess.run(command, check=False, timeout=timeout)
        return completed.returncode


@dataclass
class GraspPipelineConfig:
    frozen_selection_path: str = "logs/anygrasp_runs/frozen_selected_latest.json"
    plan_report_path: str = "logs/anygrasp_runs/full_pipeline_report_latest.json"
    execute_report_path: str = "logs/anygrasp_runs/frozen_execute_report.json"
    tcp_grasp_offset_candidates: str = "0.12,0.10,0.08,0.06"
    max_candidates: int = 50
    execute: bool = False
    move_home: bool = False
    preflight_timeout: float = 20.0
    scene_timeout: float = 10.0
    plan_timeout: float = 90.0
    execute_timeout: float = 180.0


class GraspPipelineRunner:
    def __init__(
        self,
        config: GraspPipelineConfig,
        command_runner=None,
        feedback: Optional[Callable[[str, float, str], None]] = None,
    ):
        self.config = config
        self.command_runner = command_runner or SubprocessCommandRunner()
        self.feedback = feedback

    def run(self):
        self._run_stage(
            "PREFLIGHT",
            0.05,
            self._preflight_command(),
            self.config.preflight_timeout,
        )
        self._run_stage(
            "APPLY_SCENE",
            0.15,
            self._apply_scene_command(),
            self.config.scene_timeout,
        )
        self._run_stage(
            "PLAN",
            0.65 if self.config.execute else 1.0,
            self._plan_command(),
            self.config.plan_timeout,
        )
        if self.config.execute:
            offset = self._load_selected_offset()
            self._run_stage(
                "EXECUTE",
                1.0,
                self._execute_command(offset),
                self.config.execute_timeout,
            )

    def _run_stage(self, stage: str, progress: float, command, timeout):
        self._publish_feedback(stage, progress, "started")
        try:
            return_code = self.command_runner.run(command, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            self._publish_feedback(stage, progress, "timeout")
            raise PipelineStageError(stage, f"timeout after {exc.timeout}s") from exc
        if return_code != 0:
            self._publish_feedback(stage, progress, f"failed return_code={return_code}")
            raise PipelineStageError(stage, f"command failed return_code={return_code}")
        self._publish_feedback(stage, progress, "completed")

    def _publish_feedback(self, stage: str, progress: float, description: str):
        if self.feedback is not None:
            self.feedback(stage, progress, description)

    def _preflight_command(self):
        command = ["python3", "scripts/piperx_grasp_preflight.py"]
        if self.config.move_home:
            command.append("--move-home")
        return command

    def _apply_scene_command(self):
        return ["python3", "scripts/anygrasp_apply_scene.py"]

    def _plan_command(self):
        return [
            "python3",
            "scripts/anygrasp_full_pipeline.py",
            "--stop-at-first",
            "--max-candidates",
            str(self.config.max_candidates),
            "--tcp-grasp-offset-candidates",
            self.config.tcp_grasp_offset_candidates,
            "--save-selected-path",
            self.config.frozen_selection_path,
            "--report-path",
            self.config.plan_report_path,
        ]

    def _execute_command(self, tcp_grasp_offset_z: float):
        return [
            "python3",
            "scripts/anygrasp_full_pipeline.py",
            "--frozen-selection-path",
            self.config.frozen_selection_path,
            "--tcp-grasp-offset-z",
            f"{tcp_grasp_offset_z:.3f}",
            "--execute",
            "--report-path",
            self.config.execute_report_path,
        ]

    def _load_selected_offset(self) -> float:
        path = Path(self.config.frozen_selection_path)
        try:
            snapshot = json.loads(path.read_text(encoding="utf-8"))
        except OSError as exc:
            raise PipelineStageError("EXECUTE", f"cannot read frozen selection: {path}") from exc
        if "tcp_grasp_offset_z" not in snapshot:
            raise PipelineStageError("EXECUTE", "frozen selection missing tcp_grasp_offset_z")
        return float(snapshot["tcp_grasp_offset_z"])


def build_pipeline_config(parameters: dict, execute: bool) -> GraspPipelineConfig:
    return GraspPipelineConfig(
        frozen_selection_path=parameters["frozen_selection_path"],
        plan_report_path=parameters["plan_report_path"],
        execute_report_path=parameters["execute_report_path"],
        tcp_grasp_offset_candidates=parameters["tcp_grasp_offset_candidates"],
        max_candidates=int(parameters["max_candidates"]),
        execute=execute,
        move_home=bool(parameters["move_home_before_execute"]) and execute,
    )

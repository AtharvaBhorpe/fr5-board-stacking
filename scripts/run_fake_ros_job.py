import argparse
import json
from pathlib import Path
import uuid

import rclpy

from demo_job import DESTINATIONS, SimulatedExecutor, run_job
from fake_moveit_check import RobotInterface
from fake_motion_gripper_check import GripperInterface


MOTION_STEPS = {
    "approach_source", "descend_source", "lift_source",
    "approach_profile", "descend_profile", "retreat_to_wait",
    "approach_profile_retrieve", "descend_profile_retrieve",
    "lift_profile", "approach_destination", "descend_destination",
    "retreat_destination",
}

VACUUM_STEPS = {
    "vacuum_on_source": True,
    "vacuum_off_profile": False,
    "vacuum_on_profile": True,
    "vacuum_off_destination": False,
}


class FakeRosExecutor(SimulatedExecutor):
    def __init__(self, robot, gripper, targets):
        super().__init__(fail_at=None)
        self.robot = robot
        self.gripper = gripper
        self.targets = targets

    def execute(self, step):
        if step == "check_scene":
            self.robot.require_fake_hardware()
            self.gripper.require_fake_hardware()

            # Validate all named targets before any motion or vacuum command.
            for target in set(self.targets.values()):
                self.robot.named_goal(target)

        if step in self.targets:
            self.robot.move_named(self.targets[step], execute=True)
            return {
                "confirmation": "moveit_fake_execution_succeeded",
                "target": self.targets[step],
            }

        if step in VACUUM_STEPS:
            self.gripper.set_vacuum(VACUUM_STEPS[step])
            super().execute(step)
            return {"confirmation": "fake_gripper_service_succeeded"}

        # Scene, grasp, and placement checks remain synthetic.
        # The existing two-second wait still uses a monotonic clock.
        return super().execute(step)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", required=True, choices=DESTINATIONS)
    parser.add_argument(
        "--config", type=Path,
        default=Path("config/job_fake_integration.json"),
    )
    args = parser.parse_args()

    config = json.loads(args.config.read_text())
    if config.get("purpose") != "synthetic_fake_integration_only":
        raise RuntimeError("Expected synthetic fake integration configuration")

    targets = config.get("motion_targets")
    if not isinstance(targets, dict) or set(targets) != MOTION_STEPS:
        raise RuntimeError("Configuration must map every motion step exactly")
    if not all(isinstance(v, str) and v for v in targets.values()):
        raise RuntimeError("Every motion target must be a nonempty name")

    output = Path("artifacts/jobs")
    output.mkdir(parents=True, exist_ok=True)
    path = output / f"fake-ros-{uuid.uuid4().hex}.jsonl"

    rclpy.init()
    robot = RobotInterface()
    status = 1
    try:
        gripper = GripperInterface(robot)
        executor = FakeRosExecutor(robot, gripper, targets)

        print("ROS FAKE INTEGRATION: synthetic poses and observations.")
        print(f"Log: {path.resolve()}")
        with path.open("x", encoding="utf-8") as log:
            log.write(json.dumps({
                "event": "EXECUTION_BACKEND",
                "backend": "ros_fake",
                "configuration": config,
                "observations": "synthetic",
                "physical_pick_place_validated": False,
            }) + "\n")
            log.flush()
            status = run_job(args.variant, executor, log)

        if status:
            robot.request_cancel()
        else:
            print("FAKE ROS JOB INTEGRATION PASSED")
    except (Exception, KeyboardInterrupt) as error:
        print(f"FAILED: {str(error) or type(error).__name__}")
        robot.request_cancel()
    finally:
        robot.destroy_node()
        rclpy.try_shutdown()

    raise SystemExit(status)


if __name__ == "__main__":
    main()

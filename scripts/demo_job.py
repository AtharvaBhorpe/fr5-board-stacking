import argparse
import json
from pathlib import Path
import time
import uuid


DESTINATIONS = {"VARa": 1, "VARb": 2, "VARc": 16}

STEPS = (
    "check_scene",
    "approach_source",
    "descend_source",
    "vacuum_on_source",
    "verify_source_grasp",
    "lift_source",
    "approach_profile",
    "descend_profile",
    "vacuum_off_profile",
    "retreat_to_wait",
    "wait_processing",
    "reobserve_profile",
    "approach_profile_retrieve",
    "descend_profile_retrieve",
    "vacuum_on_profile",
    "verify_profile_grasp",
    "lift_profile",
    "approach_destination",
    "descend_destination",
    "vacuum_off_destination",
    "retreat_destination",
    "verify_final_placement",
)


class SimulatedExecutor:
    def __init__(self, fail_at):
        self.fail_at = fail_at
        self.board_location = "source"
        self.vacuum_command = False

    def execute(self, step):
        # Inject a failure before the simulated action changes state.
        if step == self.fail_at:
            raise RuntimeError(f"Injected failure at {step}")

        if step == "check_scene":
            if self.board_location != "source":
                raise RuntimeError("Simulated source board unavailable")

        elif step in ("vacuum_on_source", "vacuum_on_profile"):
            expected = "source" if step.endswith("source") else "profile"
            if self.board_location != expected:
                raise RuntimeError(f"Board is not at {expected}")
            self.vacuum_command = True

        elif step in ("verify_source_grasp", "verify_profile_grasp"):
            if not self.vacuum_command:
                raise RuntimeError("Vacuum has not been commanded on")
            self.board_location = "gripper"

        elif step in ("vacuum_off_profile", "vacuum_off_destination"):
            if self.board_location != "gripper":
                raise RuntimeError("No simulated board in gripper")
            self.vacuum_command = False
            self.board_location = (
                "profile" if step.endswith("profile") else "destination"
            )

        elif step == "wait_processing":
            started = time.monotonic()
            deadline = started + 2.0
            while (remaining := deadline - time.monotonic()) > 0:
                time.sleep(remaining)
            return {"elapsed_s": time.monotonic() - started}

        elif step == "reobserve_profile":
            if self.board_location != "profile":
                raise RuntimeError("Simulated profile board missing")

        elif step == "verify_final_placement":
            if (
                self.board_location != "destination"
                or self.vacuum_command
            ):
                raise RuntimeError("Simulated final placement failed")

        # Motion actions are immediate simulated acknowledgements.
        return {"confirmation": "simulated"}


def run_job(variant, executor, log):
    started = time.monotonic()

    def emit(event, **fields):
        record = {
            "elapsed_s": round(time.monotonic() - started, 6),
            "event": event,
            "variant": variant,
            "destination_tag_id": DESTINATIONS[variant],
            "board_location_simulated": executor.board_location,
            "vacuum_command_simulated": executor.vacuum_command,
            **fields,
        }
        log.write(json.dumps(record, allow_nan=False) + "\n")
        log.flush()
        print(json.dumps(record))

    emit("JOB_STARTED", mode="simulation", hardware_connected=False)
    current_step = None
    try:
        for current_step in STEPS:
            emit("STEP_STARTED", step=current_step)
            result = executor.execute(current_step)
            emit("STEP_SUCCEEDED", step=current_step, result=result)
    except (Exception, KeyboardInterrupt) as error:
        emit(
            "JOB_FAULTED",
            step=current_step,
            reason=str(error) or type(error).__name__,
            automatic_retry=False,
        )
        return 1

    emit("SIMULATED_JOB_COMPLETED")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", required=True, choices=DESTINATIONS)
    parser.add_argument("--fail-at", choices=STEPS)
    args = parser.parse_args()

    output = Path("artifacts/jobs")
    output.mkdir(parents=True, exist_ok=True)
    path = output / f"simulation-{uuid.uuid4().hex}.jsonl"
    print(f"Simulation log: {path.resolve()}")

    with path.open("x", encoding="utf-8") as log:
        result = run_job(
            args.variant, SimulatedExecutor(args.fail_at), log
        )
    raise SystemExit(result)


if __name__ == "__main__":
    main()

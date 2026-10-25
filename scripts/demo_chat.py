import argparse
from pathlib import Path
import re
import uuid

from demo_job import STEPS, SimulatedExecutor, run_job


VARIANTS = {"a": "VARa", "b": "VARb", "c": "VARc"}


def parse_command(text):
    match = re.fullmatch(
        r"process\s+var([abc])", text.strip(), flags=re.IGNORECASE
    )
    if match is None:
        raise ValueError("Use: process VARa, process VARb, or process VARc")
    return VARIANTS[match.group(1).lower()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fail-at", choices=STEPS,
        help="Inject a failure into every simulated job in this session.",
    )
    args = parser.parse_args()
    output = Path("artifacts/jobs")
    output.mkdir(parents=True, exist_ok=True)

    print("FR5 command interface — SIMULATION ONLY")
    print("Commands: process VARa | process VARb | process VARc | quit")
    print("Each job uses a fresh simulated scene; no hardware is connected.")

    while True:
        try:
            command = input("\nYou> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nSession closed.")
            return

        if command.lower() in {"quit", "exit"}:
            return
        if not command:
            continue

        try:
            variant = parse_command(command)
        except ValueError as error:
            print(f"Rejected: {error}")
            continue

        job_id = uuid.uuid4().hex
        path = output / f"chat-simulation-{job_id}.jsonl"
        print(f"Accepted: {variant}; job={job_id}")
        print(f"Log: {path.resolve()}")

        # Synchronous execution means this interface runs one job at a time.
        with path.open("x", encoding="utf-8") as log:
            status = run_job(
                variant, SimulatedExecutor(args.fail_at), log
            )

        if status:
            print("SIMULATED JOB FAULTED. No automatic retry.")
        else:
            print("SIMULATED JOB COMPLETED.")


if __name__ == "__main__":
    main()

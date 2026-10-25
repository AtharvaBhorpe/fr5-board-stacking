"""Verify the synthetic transport-test bag without cameras or a robot."""

import argparse
import json
from pathlib import Path
import sys
import tomllib

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    args = parser.parse_args()

    with Path("config/perception.toml").open("rb") as source:
        settings = tomllib.load(source)["transport_test"]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    width, height = settings["width"], settings["height"]
    # Matches the existing synthetic publisher's constant image.
    expected_payload = bytes([42]) * (width * height * 3)
    count = 0
    previous_stamp = None

    while reader.has_next():
        topic, data, receipt_time = reader.read_next()
        if topic != settings["topic"]:
            continue

        message = deserialize_message(data, Image)
        stamp = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        assert message.header.frame_id == "transport_test_optical", "Wrong frame."
        assert (message.width, message.height) == (width, height), "Wrong size."
        assert message.encoding == "rgb8", "Wrong encoding."
        assert message.step == width * 3, "Wrong row size."
        assert message.data.tobytes() == expected_payload, "Image bytes differ."
        assert stamp > 0 and receipt_time > 0, "Missing timestamp."
        assert previous_stamp is None or stamp > previous_stamp, \
            "Image timestamps are not increasing."
        previous_stamp = stamp
        count += 1

    assert count >= settings["frames"], \
        f"Only {count} images; expected at least {settings['frames']}."

    print(json.dumps({
        "ok": True,
        "frames_verified": count,
        "payload_verified": True,
        "source_headers_preserved": True,
    }, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, OSError, RuntimeError, ValueError, KeyError) as error:
        print(json.dumps({"ok": False, "error": str(error)}))
        sys.exit(2)

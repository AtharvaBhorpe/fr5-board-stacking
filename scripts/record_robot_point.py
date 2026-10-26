"""Record a stopped robot-model frame origin from measured joints and TF. No motion."""
import argparse
from collections import deque
import json
from pathlib import Path
import time
import numpy as np


def positions_stable(values, tolerance=.003):
    a = np.asarray(values, float)
    return (a.ndim == 2 and len(a) >= 3 and np.isfinite(a).all()
            and float(np.max(np.ptp(a, axis=0))) <= tolerance)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent', default='base_link')
    parser.add_argument('--tip', default='flange')
    parser.add_argument('--label')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        assert positions_stable([[0]*6]*3)
        assert not positions_stable([[0]*6, [.1]*6, [0]*6])
        assert not positions_stable([[float('nan')]*6]*3)
        assert np.isclose(np.linalg.norm(np.array([.03,.04,0])), .05)
        print('PASS: stopped-position check, invalid-data rejection, metre distances')
        return
    if not args.label or args.output is None:
        parser.error('--label and --output are required')
    if args.output.exists():
        for line in args.output.read_text().splitlines():
            if line.strip() and json.loads(line)['label'] == args.label:
                raise ValueError('Label already exists; use a new label')
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener, TransformException
    rclpy.init()
    node = rclpy.create_node('record_robot_point')
    buffer = Buffer(cache_time=Duration(seconds=10))
    listener = TransformListener(buffer, node)
    history = deque(maxlen=1000)
    names = [f'j{i}' for i in range(1,7)]
    def callback(msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in names):
            seconds = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            history.append((seconds, msg.header.stamp, [values[n] for n in names]))
    subscription = node.create_subscription(JointState, '/joint_states', callback,
                                            qos_profile_sensor_data)
    deadline = time.monotonic()+12
    last_error = 'No measured joint feedback received'
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not history:
                continue
            latest = history[-1]
            recent = [row for row in history if row[0] >= latest[0]-1]
            now = node.get_clock().now().nanoseconds/1e9
            if (not 0 <= now-latest[0] <= .5 or len(recent) < 3
                    or recent[-1][0]-recent[0][0] < .5
                    or not positions_stable([row[2] for row in recent])):
                last_error = 'Joint feedback is stale or robot is not stationary'
                continue
            try:
                stamped = buffer.lookup_transform(args.parent, args.tip, Time.from_msg(latest[1]))
            except TransformException as error:
                last_error = str(error)
                continue
            t = stamped.transform.translation
            q = stamped.transform.rotation
            xyz = [t.x,t.y,t.z]
            quaternion = [q.x,q.y,q.z,q.w]
            if not np.isfinite(xyz+quaternion).all():
                raise ValueError('Non-finite TF')
            row = {'label': args.label, 'frame_id': args.parent, 'tip_frame': args.tip,
                   'stamp_sec': latest[1].sec, 'stamp_nanosec': latest[1].nanosec,
                   'position_m': xyz, 'quaternion_xyzw': quaternion,
                   'joint_names': names, 'joint_position_rad': latest[2],
                   'joint_range_rad': np.ptp([v[2] for v in recent],axis=0).tolist(),
                   'physical_tip_calibration_verified': False,
                   'contact_confirmed': False, 'execution_allowed': False,
                   'source': 'TF kinematics from joint feedback; verify real driver separately'}
            args.output.parent.mkdir(parents=True,exist_ok=True)
            with args.output.open('a') as stream:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
            print(json.dumps(row,indent=2))
            print('Position in mm:', [round(v*1000,3) for v in xyz])
            return
        raise RuntimeError(last_error)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

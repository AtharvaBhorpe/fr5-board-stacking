"""Validate a saved hover plan; execute only with explicit --execute."""
import argparse
import copy
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory, RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import JointState

from fake_moveit_check import RobotInterface


class ReviewedHover(RobotInterface):
    def check_and_run(self, path, execute):
        data = json.loads(path.read_text())
        result = data['response']
        if data['purpose'] != 'plan_only_horizontal_hover' or result['fraction'] != 1.0 or result['error_code']['val'] != 1:
            raise RuntimeError('Not a complete successful hover plan')
        if not 0 <= time.time() - data['created_unix'] < 600:
            raise RuntimeError('Plan expired; replan')
        trajectory = RobotTrajectory()
        set_message_fields(trajectory, result['solution'])
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if set(names) != {f'j{i}' for i in range(1, 7)} or len(names) != 6 or len(points) < 2:
            raise RuntimeError('Invalid joint trajectory')
        previous_time = -1.0
        for point in points:
            if any(len(values) != 6 or not all(math.isfinite(v) for v in values)
                   for values in (point.positions, point.velocities, point.accelerations)):
                raise RuntimeError('Invalid trajectory values')
            seconds = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if seconds <= previous_time:
                raise RuntimeError('Nonmonotonic trajectory time')
            previous_time = seconds
            # Uniform slowdown preserves path geometry and lowers both limits.
            nanoseconds = round(seconds * 1.1 * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
            point.velocities = [v / 1.1 for v in point.velocities]
            point.accelerations = [a / 1.21 for a in point.accelerations]
            if max(map(abs, point.velocities)) > 0.2 or max(map(abs, point.accelerations)) > 0.07:
                raise RuntimeError('Reduced speed/acceleration limit exceeded')
        for a, b in zip(points, points[1:]):
            if max(abs(x-y) for x,y in zip(a.positions,b.positions)) > 0.05:
                raise RuntimeError('Joint discontinuity')
        fk = self.create_client(GetPositionFK, '/compute_fk')
        validity = self.create_client(GetStateValidity, '/check_state_validity')
        if not fk.wait_for_service(timeout_sec=5) or not validity.wait_for_service(timeout_sec=5):
            raise RuntimeError('Validation services unavailable')
        target = data['target']['position']
        positions = []
        for point in points:
            state = RobotState()
            state.joint_state.name = names
            state.joint_state.position = point.positions
            state.is_diff = True
            check = GetStateValidity.Request(robot_state=state, group_name='fairino5_v6_group')
            if not self.await_result(validity.call_async(check), 5).valid:
                raise RuntimeError('Trajectory state is invalid in current planning scene')
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['tcp']
            request.robot_state = state
            response = self.await_result(fk.call_async(request), 5)
            if response.error_code.val != 1 or len(response.pose_stamped) != 1:
                raise RuntimeError('FK failed')
            position = response.pose_stamped[0].pose.position
            if abs(position.z - target['z']) > 0.005:
                raise RuntimeError('Trajectory leaves horizontal hover plane')
            positions.append(position)
        end = positions[-1]
        if math.dist([end.x,end.y,end.z], [target['x'],target['y'],target['z']]) > 0.003:
            raise RuntimeError('Endpoint does not match reviewed target')
        observed = []
        sub = self.create_subscription(JointState, '/joint_states', lambda m: observed.append(m), qos_profile_sensor_data)
        deadline = time.monotonic() + 5
        while not observed and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not observed:
            raise RuntimeError('Missing live feedback')
        message = observed[-1]
        stamp = message.header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
        current = dict(zip(message.name,message.position))
        if not -0.1 <= age <= 1 or any(n not in current or not math.isfinite(current[n]) or abs(current[n]-q)>0.005 for n,q in zip(names,points[0].positions)):
            raise RuntimeError('Robot moved or feedback stale; replan')
        print(f'VALIDATED {len(points)} collision-free states, horizontal FK, endpoint, speed and live start.', flush=True)
        if not execute:
            print('Validation only: NO EXECUTION')
            return
        client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Execution action unavailable')
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        self.active_goal = self.await_result(client.send_goal_async(goal), 5)
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError('Execution rejected')
        print('EXECUTING reviewed hover.', flush=True)
        response = self.await_result(self.active_goal.get_result_async(), 45)
        self.active_goal = None
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Execution failed: {response}')
        observed.clear()
        deadline = time.monotonic() + 3
        while not observed and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not observed:
            raise RuntimeError('Execution reported success but final feedback unavailable')
        final = dict(zip(observed[-1].name, observed[-1].position))
        error = max(abs(final[n]-q) for n,q in zip(names,points[-1].positions))
        if not math.isfinite(error) or error > 0.01:
            raise RuntimeError(f'Final joint error too large: {error}')
        print(f'HOVER EXECUTION SUCCEEDED; fresh final joint error={error:.6f} rad. No descent or vacuum.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    robot = ReviewedHover()
    try:
        robot.check_and_run(args.plan, args.execute)
    finally:
        if robot.active_goal is not None:
            robot.await_result(robot.active_goal.cancel_goal_async(), 5)
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

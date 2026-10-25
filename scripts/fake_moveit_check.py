import argparse
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


class RobotInterface(Node):
    """Named-joint motion check restricted to this project's fake hardware."""

    def __init__(self):
        super().__init__("fr5_fake_moveit_check")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.active_goal = None

    def await_result(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"ROS operation exceeded {timeout} seconds")
        return future.result()

    def read_string_parameter(self, remote_node, name):
        service = f"{remote_node.rstrip('/')}/get_parameters"
        client = self.create_client(GetParameters, service)
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"Parameter service unavailable: {service}")

            request = GetParameters.Request()
            request.names = [name]
            response = self.await_result(client.call_async(request), 10.0)

            if response is None or len(response.values) != 1:
                raise RuntimeError(f"Missing parameter: {remote_node}/{name}")

            value = response.values[0]
            if value.type != ParameterType.PARAMETER_STRING:
                raise RuntimeError(
                    f"Expected string parameter: {remote_node}/{name}"
                )
            if not value.string_value:
                raise RuntimeError(f"Empty parameter: {remote_node}/{name}")
            return value.string_value
        finally:
            self.destroy_client(client)

    def require_fake_hardware(self):
        description = self.read_string_parameter(
            "/controller_manager", "robot_description"
        )
        root = ET.fromstring(description)
        plugins = [
            (element.text or "").strip()
            for element in root.findall("./ros2_control/hardware/plugin")
        ]
        if plugins != ["mock_components/GenericSystem"]:
            raise RuntimeError(
                f"Refusing motion: expected only mock hardware, got {plugins}"
            )
        self.get_logger().info("Verified mock_components/GenericSystem")

    def named_goal(self, name):
        semantic = self.read_string_parameter(
            "/move_group", "robot_description_semantic"
        )
        root = ET.fromstring(semantic)
        states = [
            state for state in root.findall("group_state")
            if state.get("name") == name
        ]
        if len(states) != 1:
            available = [s.get("name") for s in root.findall("group_state")]
            raise RuntimeError(
                f"Named target {name!r} is not unique/available: {available}"
            )

        state = states[0]
        constraints = Constraints()
        constraints.name = name
        for joint in state.findall("joint"):
            constraint = JointConstraint()
            constraint.joint_name = joint.attrib["name"]
            constraint.position = float(joint.attrib["value"])
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        if not constraints.joint_constraints:
            raise RuntimeError(f"Named target {name} contains no joints")
        return state.attrib["group"], constraints

    def move_named(self, name, execute=False):
        self.require_fake_hardware()
        group, constraints = self.named_goal(name)

        if not self.move_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt /move_action is unavailable")

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(
            f"Target={name}, group={group}, execute={execute}, scaling=10%"
        )
        self.active_goal = self.await_result(
            self.move_client.send_goal_async(goal), 10.0
        )
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError("MoveIt rejected the goal")

        response = self.await_result(
            self.active_goal.get_result_async(), 180.0
        )
        self.active_goal = None
        result = response.result

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"Motion failed: action_status={response.status}, "
                f"moveit_error={result.error_code.val}"
            )

        points = len(result.planned_trajectory.joint_trajectory.points)
        if points == 0:
            raise RuntimeError("MoveIt returned an empty joint trajectory")

        label = "FAKE EXECUTION SUCCEEDED" if execute else "PLAN SUCCEEDED"
        print(f"{label}: target={name}, trajectory_points={points}")

    def request_cancel(self):
        if self.active_goal is not None and self.active_goal.accepted:
            try:
                self.await_result(
                    self.active_goal.cancel_goal_async(), 5.0
                )
                print("Cancellation response received; not proof of a stop.")
            except Exception as error:
                print(f"Could not confirm cancellation response: {error}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    robot = RobotInterface()
    status = 0
    try:
        robot.move_named(args.target, execute=args.execute)
    except (Exception, KeyboardInterrupt) as error:
        status = 1
        print(f"FAILED: {str(error) or type(error).__name__}")
        robot.request_cancel()
    finally:
        robot.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(status)


if __name__ == "__main__":
    main()

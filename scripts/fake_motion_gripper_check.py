import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import SetBool

from fake_moveit_check import RobotInterface


class GripperInterface:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(
            SetBool, "/gripper/set_vacuum"
        )

    def require_fake_hardware(self):
        client = self.node.create_client(
            GetParameters, "/gripper/get_parameters"
        )
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("Gripper parameter service unavailable")

            request = GetParameters.Request()
            request.names = ["use_fake_hardware"]
            response = self.node.await_result(
                client.call_async(request), 10.0
            )
            if response is None or len(response.values) != 1:
                raise RuntimeError("Missing gripper hardware-mode parameter")

            value = response.values[0]
            if (
                value.type != ParameterType.PARAMETER_BOOL
                or not value.bool_value
            ):
                raise RuntimeError("Refusing command: gripper is not fake")
            print("Verified gripper use_fake_hardware=True")
        finally:
            self.node.destroy_client(client)

    def set_vacuum(self, enabled):
        self.require_fake_hardware()
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Gripper vacuum service unavailable")

        request = SetBool.Request()
        request.data = enabled
        response = self.node.await_result(
            self.client.call_async(request), 10.0
        )
        if response is None or not response.success:
            message = (
                "No response" if response is None else response.message
            )
            raise RuntimeError(f"Vacuum command failed: {message}")
        print(f"FAKE VACUUM COMMAND ACCEPTED: {response.message}")


def main():
    rclpy.init()
    robot = RobotInterface()
    gripper = GripperInterface(robot)
    status = 0

    try:
        robot.require_fake_hardware()
        gripper.require_fake_hardware()

        # Confirm the target can be planned before changing vacuum state.
        robot.move_named("pos2", execute=False)

        gripper.set_vacuum(True)
        robot.move_named("pos2", execute=True)
        gripper.set_vacuum(False)
        print("FAKE MOTION + GRIPPER CHECK PASSED")
    except (Exception, KeyboardInterrupt) as error:
        status = 1
        print(f"CHECK FAILED: {str(error) or type(error).__name__}")
        robot.request_cancel()
        print("No automatic vacuum change or retry after failure.")
    finally:
        robot.destroy_node()
        rclpy.try_shutdown()

    raise SystemExit(status)


if __name__ == "__main__":
    main()

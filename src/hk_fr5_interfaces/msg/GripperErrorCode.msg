# Result code of /gripper/grip and /gripper/release (decision-log 5l, W-12).
# Same pattern as moveit_msgs/MoveItErrorCodes: compare val with the constants. 0 = not set.
uint8 SUCCESS=1
uint8 NO_VACUUM=2           # grip: no vacuum achieved (DI0) within the timeout, vacuum pump switched off
uint8 CANCELLED=3           # goal cancelled by the caller (grip: vacuum pump switched off)
uint8 PREEMPTED=4           # grip interrupted by a release goal
uint8 COMM_FAILURE=5        # XML-RPC to the robot controller unreachable or timed out
uint8 CONTROLLER_ERROR=6    # robot controller returned an error code (see message)
uint8 INVALID_GOAL=7        # bad goal field, e.g. negative timeout

uint8 val

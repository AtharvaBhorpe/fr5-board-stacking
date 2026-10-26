# Gripper I/O and state, topic /gripper/state (latched; state_rate + on every change).
# I/O: vacuum pump = tool DO0, blow-off pump = tool DO1, vacuum achieved = tool DI0 (decision-log 5l).
# DO values = last commanded by gripper_controller (no read-back: GetToolDO crashed the controller, H-2);
# the node switches both pumps off at start (H-7), never both on (H-8).
string IDLE="IDLE"                          # nothing held, pumps off
string GRIPPING="GRIPPING"                  # vacuum pump on, waiting for vacuum achieved
string HOLDING="HOLDING"                    # vacuum achieved, pump stays on
string RELEASING="RELEASING"                # vacuum pump off, blow-off pulse running
string PART_LOST="PART_LOST"                # vacuum lost while HOLDING; pump left on, caller decides
string COMM_ERROR="COMM_ERROR"              # XML-RPC unreachable or timed out
string CONTROLLER_ERROR="CONTROLLER_ERROR"  # robot controller returned an error code
# A new grip / release goal clears PART_LOST, COMM_ERROR and CONTROLLER_ERROR.

std_msgs/Header header
bool vacuum_on              # DO0, last commanded
bool blow_off_on            # DO1, last commanded
bool vacuum_achieved        # DI0 (debounced)
string state                # one of the constants above
string message              # last event, human-readable

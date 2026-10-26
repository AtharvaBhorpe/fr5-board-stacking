# World model: the single source of facts for the main controller, the dashboard and the LLM (/task/world, latched).
std_msgs/Header header      # stamp = last update, frame_id = base_link
bool vision_ok              # fresh Vision 1 detections
float64 vision_age          # seconds since the last detection message
Board[] boards
Zone[] zones
Obstacle[] obstacles
string held_board_id        # board on the cup ("" = nothing held)
string gripper_state        # last /gripper/state (e.g. IDLE, HOLDING, ERROR)

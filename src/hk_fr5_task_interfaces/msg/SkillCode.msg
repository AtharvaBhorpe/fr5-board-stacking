# Result code of every skill (control-loop.md section 4 / 6). One shared list so the main controller and the LLM
# can react the same way to every skill.
uint16 SUCCESS=0
uint16 CANCELLED=1          # cancelled by the main controller / operator (smooth stop)
uint16 INVALID_REQUEST=2    # bad arguments (unknown board / zone name, bad numbers)
uint16 BOARD_NOT_FOUND=10   # board unknown or not seen recently (stale)
uint16 BOARD_MOVED=11       # board moved during the approach
uint16 NOT_HOLDING=12       # place / release without a board on the cup
uint16 ALREADY_HOLDING=13   # pick while a board is held
uint16 TARGET_UNKNOWN=14    # target zone not detected / not configured
uint16 TARGET_OCCUPIED=15   # processing zone not EMPTY
uint16 STACK_FULL=16        # drop-off stack at its max count / height
uint16 VISION_STALE=17      # no fresh detections from Vision 1
uint16 PLAN_FAILED=20       # MoveIt found no plan (after the skill's own retry)
uint16 MOTION_FAILED=21     # execution failed / aborted
uint16 OBSTACLE=22          # hand / unknown object in the work area (motion stopped)
uint16 GRIP_FAILED=30       # no vacuum within the timeout (after the skill's own retry)
uint16 LOST_ON_LIFT=31      # vacuum lost right after lifting
uint16 LOST_IN_TRANSIT=32   # vacuum lost while moving: board dropped somewhere
uint16 RELEASE_FAILED=33    # board still on the cup after release
uint16 NOT_VERIFIED=34      # camera does not confirm the result (board not seen where expected)
uint16 GRIPPER_ERROR=35     # gripper node error / not available
uint16 SCENE_ERROR=36       # /scene/* call failed
uint16 TIMEOUT=40           # wait / skill timed out
uint16 INTERNAL_ERROR=50

uint16 value

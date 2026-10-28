# One zone (Vision 1 or a config fallback). base_link, metres / radians.
string name                 # "processing_1" | "processing_2" | "drop_off_1" | "drop_off_2" | "fault"
string kind                 # "processing" | "drop_off" | "fault"
string state                # processing: EMPTY | LOADED | PROCESSING | DONE | FAULT; others: OK | FULL
float64 x                   # zone centre
float64 y
float64 yaw
float64 length
float64 width
string board_id             # processing: the board that belongs to this zone ("" when EMPTY)
int32 stack_count           # drop_off / fault: boards placed (by us)
float64 stack_top_z         # top surface z of the stack (base_z when empty)
float64 base_z              # zone surface z: top of the zone sheet (Vision 1 top_z / config z; default table top)
bool fixed                  # true = from config (CL-15 fallback), false = detected by Vision 1
float64 age                 # seconds since last seen (fixed: 0)
float64 state_age           # seconds in the current state
string message              # e.g. FAULT reason

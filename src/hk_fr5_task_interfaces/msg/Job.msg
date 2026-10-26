# One queued job (control-loop.md section 5, presets CL-12; the LLM uses the same types, CL-18).
# type:
#   "process"    pick boards (select_*) -> place in `zone` (processing) -> wait until processed -> place in `to`
#   "transfer"   pick boards (select_*) -> place in `to`            (single board: select_board, count 1)
#   "clear_zone" pick everything in `from_zone` -> place in `to`
#   "home"       move home
string id                   # assigned by the main controller ("" when adding)
string type
string select_variant       # "" = any variant
string select_board         # world-model board id; "" = choose by variant / from_zone
string from_zone            # "" = input area
string zone                 # processing zone for "process"
string to                   # target zone
int32 count                 # boards to handle; 0 = all that match
int32 priority              # higher runs first (ties: queue order); changeable at runtime (Control set_priority, CL-22)
string source               # "dashboard" | "llm" | "cli"
string state                # QUEUED | RUNNING | WAITING | PAUSED | DONE | FAILED | CANCELLED
int32 done_count
int32 failed_count          # boards skipped (problem) or sent to the fault zone
string message              # last result / reason

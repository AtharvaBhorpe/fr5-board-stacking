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
int32 count                 # target number of SUCCESSFUL boards (delivered); 0 = all that match (S1);
                            #   -1 = continuous: never ends by itself, waits for new matching boards (cancel to stop, TM-39)
int32 max_attempts          # stop starting new boards after this many attempts; 0 = no limit (S1)
int32 priority              # higher runs first (ties: queue order); changeable at runtime (Control set_priority, CL-22)
string source               # "dashboard" | "llm" | "cli"
string state                # QUEUED | RUNNING | WAITING | PAUSED | DONE | CANCELLED (DONE may be partial: see message)
int32 attempts              # boards this job started on (picked, or skipped after a failed pick)
int32 delivered             # boards that reached their final target (process: `to`, or processed in the zone if no `to`)
int32 to_fault              # boards put into the fault zone (not processed, target full, stop while holding, release_safe)
int32 lost                  # boards lost on the way (LOST_ON_LIFT, LOST_IN_TRANSIT, NOT_HOLDING)
int32 skipped               # boards left alone for good (GRIP_FAILED, not found after retries, motion failures)
string message              # last result / reason

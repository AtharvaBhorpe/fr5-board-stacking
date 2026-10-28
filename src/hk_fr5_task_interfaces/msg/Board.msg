# One board as the world model knows it. base_link, metres / radians.
string id                   # world-model id, stable across frames (e.g. "b7"); not the Vision 1 track id
string variant              # ORIGINAL variant, kept through processing (CL-19): "white_small" | "white_large" | "red" | "unknown"
string label                # raw Vision 1 class right now (e.g. "red", or "other" = non-variant board, CL-19)
bool processed              # set by the world model when the board came back into its processing zone (CL-19)
string location             # "input" (input area / input stack) | "zone" | "held"
string zone                 # zone name when location == "zone", else ""
float64 x                   # board centre
float64 y
float64 yaw                 # long side direction, (-pi/2, pi/2]
float64 length              # measured from the image (long side)
float64 width
float64 thickness           # from the variant config (CL-17)
float64 top_z               # estimated top surface z (table / stack); NaN = unknown (input stack: contact search)
float64 score               # detection confidence 0..1
bool inside                 # centre inside the calibrated tag area
bool size_ok                # Vision 1 variant label agrees with the measured size (CL-9)
float64 age                 # seconds since last seen

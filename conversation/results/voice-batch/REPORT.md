# Counted speech and narration verification

The reported transcript "Pick two variant B volts" was rejected as unsupported; it was not
silently converted to one pick. The following "Pick variant B" was a separate single-pick request.

Both speech input endpoints now share counted-board vocabulary and beam size 5. A voice-only,
exact-command repair changes a trailing "volts" or "bolts" to "boards" without changing count
or variant. Negated, embedded, unrelated, and unsupported commands remain unchanged. Repairs
show both original and interpreted text in chat.

Batch narration uses first/second/etc. and suppresses duplicate controller announcements.
FIFO speech playback prevents progress messages from cutting each other off; epoch changes
interrupt stale speech. 101 unit tests passed, including repair boundaries, narration order,
FIFO delivery, and interruption.

Six synthetic Phonon speech samples (normal and quiet levels for counted B, counted A, and
sort-all commands) passed actual local Whisper recognition and bounded intent parsing: asr.json.
A synthesized two-B command passed actual ASR, Gemma image checks, GPU vision, and two MoveIt
mock pick/place cycles: ../batch-moveit/voice-counted.json. Duration was 45.46 seconds.
The observed chat sequence included first-board pick, first-board placement, second-board pick,
and final placement/completion, with no speech error event.

Synthetic speech does not validate the user's microphone, accent, or background noise. A human
PTT retry is still required. Tests were simulation-only on domain 62; no real robot was commanded.

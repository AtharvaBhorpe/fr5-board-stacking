# Parakeet demo validation — 2026-10-27

Model: nvidia/parakeet-tdt-0.6b-v3, revision 541d1f99c6b0c3cd0b11a95167540bb8edefd82b.
Runtime: isolated Pixi speech environment, CUDA BF16. Existing CUDA/PyTorch files
are shared with train using verified identical hard links; 8.61 GiB reclaimed.

- 102 unit tests pass.
- Six saved synthetic command recordings pass after bounded voice normalization.
- Raw variant A transcripts join “A boards” into “aboards”; the exact spacing repair
  preserves the spoken count/variant and rejects negated or incomplete commands.
- Two seconds of silence produces empty text through the retained speech detector.
- Warm live transcription HTTP times: 63–110 ms for these short recordings,
  with the simulation running. This is not microphone-to-response latency.
- Actual Parakeet transcript -> Gemma scene gate -> GPU vision -> MoveIt mock
  hardware completed two variant B cycles in 44.35 seconds.
- Human microphone/accent accuracy and real hardware are not validated here.

Evidence: parakeet-live.json; ../batch-moveit/parakeet-voice-counted.json.
The prior Whisper baseline is retained in whisper-baseline.json.

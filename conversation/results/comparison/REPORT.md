# Local VLM and Phonon comparison — 2026-10-27

Follow-up: [Gemma QAT/llama.cpp retry and live Phonon](GEMMA-QAT.md) substantially improves the Gemma
deployment results and replaces the live eSpeak output. This file preserves the earlier configuration's evidence.

Recommendation: keep Qwen as the current MVP baseline; do not replace its visual gate with this Gemma configuration. Phonon 31M English is a promising local speech replacement for either model. Speech was tested through saved VLM replies, not integrated into the live push-to-talk service yet.

## What ran

- Same 12 image/question fixtures, repeated three times per model; 36 turns each, zero final-run inference errors. Repeats are not independent examples.
- Qwen3-VL-2B-Instruct BF16, Transformers 4.57.6, existing HTTP service.
- Gemma 4 E2B IT, revision `3e22461f65e89153144f8adb70e3b8c2cc9845a7`, Transformers 5.21.0, bitsandbytes 0.50.2 NF4 with BF16 compute. Image/language modules on GPU; unused audio modules CPU-offloaded. Loading allocator preallocation disabled in the benchmark process after an OOM. Entirely GPU-resident attempts failed from insufficient memory.
- Images limited to 640×480 and JPEG quality 90 for both. Model-specific vision token budgets differ (Qwen maximum 256; Gemma 280). Same prompts, deterministic decoding, maximum 110 new tokens, thinking disabled for Gemma, no conversation history.
- Both final VLM runs had RViz, MoveIt demo and vision inference stopped. No robot commands were dispatched. This is a practical configuration comparison, not an isolated architecture comparison: precision, runtimes and transport differ.

## VLM results

| Measure | Qwen | Gemma |
|---|---:|---:|
| Median full reply | 0.465 s | 0.801 s |
| p95 full reply | 0.545 s | 1.115 s |
| Correct gate decisions | 12/18 | 6/18 |
| Positive fixtures approved | 3/9 | 6/9 |
| Negative fixtures wrongly approved | 0/9 | 9/9 |
| Reported allocated model memory | 4,059 MiB | 6,301 MiB |

Gemma peak allocated memory was 6,468 MiB. This has very little room for concurrent GPU vision on this laptop; these figures exclude some driver/runtime allocations. Qwen memory is the service's startup allocation, not peak inference memory.

The gate set has three visible-target fixtures and three negative fixtures, each repeated three times. Gemma accepted blank images and an image containing an instruction despite the image contradicting the supplied detections. Qwen rejected the negatives, but incorrectly rejected the visible A and B fixtures. Neither result establishes general visual reliability. Both receive variant labels in structured detections; these tests do not establish independent visual variant recognition.

Manual review of the six conversational cases:

- Scene description: Qwen invents an ongoing handling action; Gemma gives a broadly grounded description.
- Ambiguous pick: Qwen chooses a board; Gemma asks which board, but does not explicitly ask A/B/C.
- Hidden sheet count: both fabricate a count (Qwen two; Gemma three).
- Failed pick: both correctly report vacuum failure from supplied state.
- Waiting for confirmation: Qwen falsely claims placement completed. Gemma says no, but fails to explain the waiting state.
- Exact cup gap: Qwen fabricates 100 mm; Gemma says the measurement is unavailable.

These are raw model replies. The existing bridge independently handles some ambiguity and task-state questions; model prose cannot dispatch motions. This test does not bypass or validate the entire controller stack.

## Phonon speech results

Both sets of six conversational replies were synthesized verbatim using the same model and default voice:

- `ptts==0.5.0`, CPU, four threads, q8, seed 42; 24 kHz mono PCM WAV.
- `gradium/phonon-31m-en@8c64b55ec301728e2d1695bfea87a08a267d0c2d`.
- One excluded speech warmup; one timed synthesis per reply. Different texts mean these are not voice-quality or VLM-dependent TTS speed rankings.

| Measure | Qwen reply set | Gemma reply set |
|---|---:|---:|
| Generated non-silent WAV files | 6/6 | 6/6 |
| Median first PCM chunk | 14.1 ms | 17.6 ms |
| First PCM chunk range | 10.9–29.7 ms | 14.5–19.9 ms |
| Audio generation speed | 16.9–19.4× real time | 17.0–20.0× real time |

First-chunk timing excludes model load, VLM generation, recording, ASR, sound-device buffering and playback. The metrics' summed VLM-plus-first-audio values are estimates from separate runs, not measured end-to-end conversation latency. No GPU was used for Phonon.

All WAVs were checked for finite, nonzero samples. A local Whisper base.en transcription smoke check recovered the main content of all 12 clips, with a `yellow-bordered` / `yellow-boarded` discrepancy and spelling/format normalization. This is not a human listening evaluation or proof of TTS accuracy. See `phonon/transcription-check.json`. Unsupported claims remain audible in the intentionally unfiltered benchmark clips.

Listen to the correct failure responses: [Qwen + Phonon](phonon/qwen/failed_task.wav), [Gemma + Phonon](phonon/gemma/failed_task.wav). All replies and timings are in each model's `metrics.json`.

## Reproduce

From the worktree root, use the isolated environments created for this test (these `/tmp` environments will not survive reboot):

```bash
# Qwen service must be running; no demo or hardware is needed.
pixi run -e train python conversation/compare_gemma.py qwen

# Stop Qwen before loading Gemma on the same GPU.
HF_HOME=/home/ub26/.cache/robot-conversation-comparison /tmp/gemma-comparison-env/bin/python conversation/compare_gemma.py gemma

# Works from saved results; no VLM needs to be running.
/tmp/phonon-env/bin/python conversation/compare_phonon.py conversation/results/comparison/qwen.json conversation/results/comparison/gemma.json
```

Phonon environment: standalone Python venv with `ptts==0.5.0` and NumPy. Gemma environment: Python 3.12 venv inheriting the Pixi train packages, with Transformers 5.21.0 and bitsandbytes 0.50.2 installed locally. The current Pixi runtime was not upgraded. Gemma weights are cached under `~/.cache/robot-conversation-comparison`; Phonon uses its pinned default Hub cache.

Phonon 31M English weights are Apache-2.0. The 40M English/German/multilingual models are gated and CC BY-NC 4.0; they were not tested. Native Gemma audio input, German, live microphone/noise, speaker playback, speech interruption, concurrent vision performance, and real hardware remain untested here. The MVP still uses eSpeak until a separate integration validates streaming playback and cancellation.

Sources: [Phonon runtime and licensing](https://github.com/gradium-ai/xn-ptts), [31M model](https://huggingface.co/gradium/phonon-31m-en), [Gemma E2B model](https://huggingface.co/google/gemma-4-E2B-it).

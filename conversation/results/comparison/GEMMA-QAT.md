# Gemma optimized-runtime retry and live Phonon

2026-10-27. The previous memory conclusion was too broad: the Transformers NF4 deployment was not representative of optimized edge runtimes. Gemma is viable on this laptop with its vision pipeline. Runtime feasibility and grounded visual decisions are separate results.

## Runtime and unchanged-fixture results

Tested Google's `gemma-4-E2B-it-qat-q4_0-gguf`, revision `675cff42a74c774d6cb76f76d8eacb49b48c9b93`, with its matching multimodal projector. Runtime: llama.cpp b11934, commit `8542e9435`, CUDA 12.8 build. Context 8192, one slot, four CPU threads, GPU layers 99, image token budget 280. No custom Python embedding wrapper. Thinking and cross-request prompt-cache reuse disabled. Same prompts, JPEG-decoded image pixels and 110-token limit as the previous comparison; the adapter sends the decoded pixels losslessly as PNG to llama.cpp.

| Configuration | Median reply | p95 reply | Gate decisions correct | Negative cases wrongly approved |
|---|---:|---:|---:|---:|
| Qwen BF16 / Transformers | 0.465 s | 0.545 s | 12/18 | 0/9 |
| Gemma NF4 / Transformers | 0.801 s | 1.115 s | 6/18 | 9/9 |
| **Gemma QAT Q4_0 / llama.cpp** | **0.265 s** | **0.348 s** | **15/18** | **3/9** |

Each has 12 fixtures repeated three times; these are not 36 independent examples. QAT correctly approved all nine positive trials. Its three failures were repeats of the blank-image VARc case: it trusted the claimed detection despite the blank image. It rejected blank VARb and the image-instruction fixture. It still invented three hidden sheets, did not clarify the ambiguous pick, and added an unsupported destination claim to the waiting-state answer. It correctly reported vacuum failure and declined an exact metric gap.

This changes the practical assessment: QAT/llama.cpp is a strong deployment candidate, much faster and more memory-efficient than our first Gemma configuration. It is still not a reliable independent image approval gate. Both checkpoint and runtime changed, so their individual contributions are not isolated. Defaults were not switched to Gemma automatically.

Observed llama-server GPU allocation was 2,702 MiB after loading and 2,798 MiB during the simulation snapshot, with 650 MiB for GPU vision. These are process snapshots, not peak-memory measurements; host RAM is additional. The weight and projector files are approximately 3.35 GB and 0.99 GB on disk. The desktop setup does not reproduce the Pi's memory figure.

## Vision + MoveIt retry

All three chat → Gemma QAT image check → RViz panel → MoveIt mock pick/place cases passed:

| Variant | QAT retry | Previous matched Qwen |
|---|---:|---:|
| A | 27.40 s | 28.54 s |
| B | 20.73 s | 20.91 s |
| C | 24.06 s | 22.90 s |

Recorded composite image, mock hardware only, automated confirmations after the runner verifies SIMULATION. One run per variant, not a statistically meaningful task-speed ranking. Vision delivered 200 messages over 40 seconds: 5.001 Hz, maximum gap 0.241 s. All ten refusal/status checks dispatched zero tasks; action cancellation returned a controller result and duplicate task ID replay was suppressed. See `../sim-comparison/gemma-qat/` for evidence. These checks do not validate physical grasping or live-camera tracking.

## Live Phonon replacement

The user correctly identified that the interactive demo was still using eSpeak. `model_server.py` now uses the persistent `PhononSpeech` engine for every backend. The pinned 31M English q8 model synthesizes on CPU; a worker streams 24 kHz float audio to `pacat`. New speech supersedes old speech. `/silence`, clear, and PTT-start share the interruption path. Synthesis does not hold the VLM inference lock. There is no eSpeak fallback.

`ptts==0.5.0` is tracked in Pixi's train feature and lockfile. eSpeak remains only for the existing synthetic-ASR fixture generator. First Phonon model use may download its pinned weights; subsequent synthesis is local.

Live playback, interruption and utterance replacement passed via `check_speech.py`; observed stop-to-idle was 0.089 s in that run. Health reported no speech errors. No microphone was captured. This checks software playback completion and interruption, not human-rated voice quality or measured acoustic latency. Actual RViz PTT microphone interaction remains untested end to end. The QAT refusal and cancellation runs were repeated with live Phonon; the initial three matched QAT pick runs preceded the voice replacement.

Saved QAT reply WAVs are under `phonon/llama/`. The live-Phonon VARb integration rerun passed in 20.67 s;
`../sim-comparison/gemma-qat/phonon-varb.json` records it separately, preserving the initial timing comparison.

## Reproduce and edge-device context

See `../../README.md` for `gemma_qat.sh`, `model.sh --backend llama`, and mock demo launch commands. Run `pixi run -e train python conversation/compare_gemma.py llama` for the unchanged fixtures and `pixi run -e train python conversation/check_speech.py` for the audible playback checks.

Google reports Gemma 4 E2B on Raspberry Pi 5 with LiteRT-LM QAT at about 1,432 MB peak memory and 9 decode tokens/s in its specified benchmark. That is a different runtime, artifact and platform; we did not test a Pi or phone. The successful laptop GGUF run confirms that the earlier Transformers footprint should not be generalized to all Gemma deployments.

Sources: [Google's Raspberry Pi benchmark](https://developers.googleblog.com/mastering-edge-ai-on-raspberry-pi-with-litert-and-gemma/), [Google's QAT model card](https://huggingface.co/google/gemma-4-E2B-it-qat-q4_0-gguf), [llama.cpp b11934](https://github.com/ggml-org/llama.cpp/releases/tag/b11934), [Phonon runtime](https://github.com/gradium-ai/xn-ptts).

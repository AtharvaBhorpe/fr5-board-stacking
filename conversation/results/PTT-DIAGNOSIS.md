# Push-to-talk diagnosis — 2026-10-27

Two distinct issues were found.

1. `parec` used its default capture buffering. A one-second recording delivered zero samples; requesting
   30 ms latency and 10 ms processing delivered 0.93 s. The live recorder now explicitly uses raw float32
   at 16 kHz with those low-latency settings. Recorder errors are no longer discarded, and health exposes
   capture duration and level statistics without storing microphone audio.
2. The user's subsequent live attempt captured 1.95 s during a 1.994 s hold, but still produced no speech.
   Peak 0.02814 and RMS 0.02553 indicated a strongly DC-dominated signal. Independent stereo capture from
   the selected HDA Digital Microphone measured a mean near -0.0255 on both channels. ALSA also exposes
   `acppdmmach` (ACP PDM DMIC), absent from the normal PipeWire source list. Testing it directly at its
   supported 48 kHz / S32_LE / stereo format returned values nearly stuck at -1.0. A temporary PulseAudio
   ALSA source reproduced this. The source was unloaded and the original default source restored.

The input is not muted in PulseAudio. Hardware/firmware mute or the kernel audio path remains unresolved;
no kernel, system driver configuration or firmware changes were made. Host: Lenovo 83F1 / Legion 5
15AKP10, kernel 7.0.0-38-generic. The default source is
`alsa_input.pci-0000_65_00.6.HiFi__Mic1__source`. Its volume was left at the user's 29% setting.

An isolated virtual-source test of the **actual HTTP PTT capture → Whisper pipeline** passed: a saved
Phonon WAV was played into a temporary null sink, captured for 3.3 s, and recognized as
“The pick failed because the vacuum failed.” No hardware microphone content was captured by that test,
and the temporary sink/server were removed. Evidence: `ptt-loopback.json`. The normal UI path still
requires a successful user microphone retry; this virtual test does not prove physical mic health.

Empty-transcript feedback now distinguishes insufficient capture from an unrecognized or nearly constant
input signal. Speech detection safeguards were not relaxed. The 65 existing policy/service tests pass.

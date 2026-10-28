"""Speak saved VLM comparison answers locally; never sends robot commands."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import time
import wave

import numpy as np
import ptts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path, nargs="+")
    parser.add_argument("--model", default=ptts.DEFAULT_MODEL)
    args = parser.parse_args()
    ptts.set_num_threads(4)
    start = time.perf_counter()
    tts = ptts.TTS(args.model, lang="en", device="cpu", quant="q8")
    metadata = {"model": args.model, "runtime": importlib.metadata.version("ptts"),
                "device": "cpu", "threads": 4, "quant": "q8", "voice": "default",
                "sample_rate": tts.sample_rate, "load_s": time.perf_counter()-start}
    tts.synth("Speech warmup.")
    for source in args.results:
        data = json.loads(source.read_text())
        output = source.parent / "phonon" / source.stem
        output.mkdir(parents=True, exist_ok=True)
        rows = []
        for case in data["cases"]:
            if case["repeat"] != 0 or not case.get("rubric"):
                continue
            text = case["result"].get("answer", "")
            if not text:
                continue
            started = time.perf_counter()
            first = None
            chunks = []
            with tts.stream(text, seed=42) as audio:
                for chunk in audio:
                    if len(chunk):
                        if first is None:
                            first = time.perf_counter()-started
                        chunks.append(chunk.copy())
            elapsed = time.perf_counter()-started
            pcm = np.concatenate(chunks)
            if not np.isfinite(pcm).all() or not np.any(pcm):
                raise RuntimeError("Invalid or silent synthesized audio")
            path = output / (case["case"] + ".wav")
            with wave.open(str(path), "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(tts.sample_rate)
                wav.writeframes((np.clip(pcm, -1, 1)*32767).astype("<i2").tobytes())
            row = {"case": case["case"], "text": text, "first_chunk_s": first,
                   "synthesis_s": elapsed, "audio_s": len(pcm)/tts.sample_rate,
                   "vlm_s": case["seconds"], "wav": str(path),
                   "estimated_vlm_plus_first_audio_s": case["seconds"]+first}
            rows.append(row)
            print(json.dumps(row), flush=True)
        (output / "metrics.json").write_text(json.dumps({"metadata": metadata, "cases": rows}, indent=2)+"\n")


if __name__ == "__main__":
    main()

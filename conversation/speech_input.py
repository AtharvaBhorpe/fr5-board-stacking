"""Shared English ASR vocabulary and conservative command transcription repair."""
import re

from policy import batch_request

PROMPT = (
    "Robot workcell vocabulary: variant A, variant B, variant C; board, boards. "
    "Move two variant B boards to drop off b. Process variant A in processing one then to drop off a. "
    "Clear drop off a to drop off b. Sort all visible boards. Status. Pause. Resume. Stop. Cancel batch. What do you see?"
)


def transcribe_audio(asr, audio):
    if isinstance(asr, ParakeetRecognizer):
        return asr.transcribe(audio)
    segments, _ = asr.transcribe(audio, language="en", beam_size=5, vad_filter=True,
                                 initial_prompt=PROMPT)
    return " ".join(s.text for s in segments).strip()


class ParakeetRecognizer:
    model_id = "nvidia/parakeet-tdt-0.6b-v3"
    revision = "541d1f99c6b0c3cd0b11a95167540bb8edefd82b"

    def __init__(self, *, device="cuda", download=False, cache_dir=None):
        import torch
        from transformers import AutoModelForTDT, AutoProcessor

        options = dict(local_files_only=not download, cache_dir=cache_dir, revision=self.revision)
        self.processor = AutoProcessor.from_pretrained(self.model_id, **options)
        self.model = AutoModelForTDT.from_pretrained(
            self.model_id, dtype=torch.bfloat16 if device == "cuda" else torch.float32,
            device_map=device, **options).eval()

    def transcribe(self, audio):
        import torch
        from faster_whisper.vad import get_speech_timestamps

        if len(audio) < 1600 or not get_speech_timestamps(audio):
            return ""
        with torch.inference_mode():
            inputs = self.processor(audio, sampling_rate=16000, return_tensors="pt")
            inputs = inputs.to(self.model.device, dtype=self.model.dtype)
            output = self.model.generate(**inputs, return_dict_in_generate=True)
            return self.processor.batch_decode(output.sequences, skip_special_tokens=True)[0].strip()


def normalize_voice_command(text):
    """Repair observed noun/spacing errors only in an otherwise exact counted pick command."""
    corrected = re.sub(r"\b(?:volts|bolts)(?=\s*(?:please)?[.!?]*$)", "boards", text, flags=re.I)
    corrected = re.sub(r"\bvariant\s+([abc])(boards?)\b", r"variant \1 \2", corrected, flags=re.I)
    return corrected if batch_request(corrected) else text

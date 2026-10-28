"""Speech recognition (Parakeet, models env) and the conservative repair of heard commands (any env)."""
import re

from policy import batch_request


class ParakeetRecognizer:
    model_id = "nvidia/parakeet-tdt-0.6b-v3"
    revision = "541d1f99c6b0c3cd0b11a95167540bb8edefd82b"

    def __init__(self, device="cuda"):
        import torch
        from transformers import AutoModelForTDT, AutoProcessor

        self.processor = AutoProcessor.from_pretrained(self.model_id, revision=self.revision)
        self.model = AutoModelForTDT.from_pretrained(
            self.model_id, revision=self.revision, dtype=torch.bfloat16 if device == "cuda" else torch.float32,
            device_map=device).eval()

    def transcribe(self, audio):
        import torch
        from faster_whisper.vad import get_speech_timestamps

        if len(audio) < 1600 or not get_speech_timestamps(audio):  # no speech: Parakeet would invent words
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

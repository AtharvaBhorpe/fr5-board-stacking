"""Persistent local Phonon synthesis with interruptible PulseAudio playback."""
import subprocess
import threading
import re

MODEL = "hf://gradium/phonon-40m-multilingual@1537ce46c64f42903e0d608b132ccb073dac46d2"


def spoken_text(text):
    return re.sub(r"\bVAR([abc](?:/[abc])*)\b",
                  lambda m: ", ".join("variant " + letter.upper() for letter in m.group(1).split("/")),
                  str(text), flags=re.IGNORECASE)


class PhononSpeech:
    def __init__(self):
        import ptts
        ptts.set_num_threads(4)
        self.tts = ptts.TTS(MODEL, lang="en", device="cpu", quant="q8")
        self.tts.synth("Ready.")
        self.lock = threading.RLock()
        self.synthesis_lock = threading.Lock()
        self.epoch = 0
        self.player = None
        self.worker = None
        self.error = None

    def stop(self):
        with self.lock:
            self.epoch += 1
            player = self.player
            if player is not None and player.poll() is None:
                player.terminate()
                try:
                    player.wait(timeout=.2)
                except subprocess.TimeoutExpired:
                    player.kill()
                    player.wait(timeout=.2)

    def speak(self, text):
        with self.lock:
            self.stop()
            self.error = None
            self.worker = threading.Thread(target=self._play, args=(spoken_text(str(text)[:500]), self.epoch), daemon=True)
            self.worker.start()

    def status(self):
        with self.lock:
            return {"engine": "phonon", "model": MODEL, "device": "cpu", "voice": "default",
                    "active": self.worker is not None and self.worker.is_alive(),
                    "playing": self.player is not None and self.player.poll() is None,
                    "error": self.error}

    def _play(self, text, epoch):
        player = None
        try:
            with self.synthesis_lock:
                with self.lock:
                    if epoch != self.epoch or not text.strip():
                        return
                    player = subprocess.Popen([
                        "pacat", "--playback", "--raw", "--format=float32le",
                        f"--rate={self.tts.sample_rate}", "--channels=1", "--latency-msec=50",
                        "--client-name=Robot Phonon"], stdin=subprocess.PIPE,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self.player = player
                with self.tts.stream(text) as audio:
                    for chunk in audio:
                        with self.lock:
                            if epoch != self.epoch:
                                break
                        player.stdin.write(chunk.astype("<f4", copy=False).tobytes())
                player.stdin.close()
                player.wait(timeout=60)
                with self.lock:
                    if epoch == self.epoch and player.returncode != 0:
                        self.error = f"Audio player exited with code {player.returncode}"
        except Exception as exc:
            with self.lock:
                if epoch == self.epoch:
                    self.error = str(exc)[:200]
        finally:
            if player is not None:
                if player.poll() is None:
                    player.kill()
                    player.wait(timeout=1)
                with self.lock:
                    if self.player is player:
                        self.player = None

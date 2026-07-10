
import os
import tempfile
import subprocess
from openai import OpenAI


class TTS:
    def __init__(self, openai_api_key, voice="nova"):
        """
        voice 추천:
        - nova: 밝고 자연스러운 여성 느낌
        - shimmer: 부드러운 여성 느낌
        - alloy: 중성 느낌
        """
        self.client = OpenAI(api_key=openai_api_key)
        self.voice = voice

    def speak(self, text):
        if text is None or text.strip() == "":
            return

        text = text.strip()

        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as temp_audio:
                audio_path = temp_audio.name

            response = self.client.audio.speech.create(
                model="tts-1",
                voice=self.voice,
                input=text,
            )

            response.stream_to_file(audio_path)

            subprocess.run(
                ["mpg123", "-q", audio_path],
                check=False,
            )

        except Exception as e:
            print(f"[TTS ERROR] {e}")

        finally:
            try:
                if os.path.exists(audio_path):
                    os.remove(audio_path)
            except Exception:
                pass
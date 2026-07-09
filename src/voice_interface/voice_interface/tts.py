
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

from openai import OpenAI
import io
import sounddevice as sd
import scipy.io.wavfile as wav


class TTS:
    def __init__(self, openai_api_key, voice="alloy", model="tts-1"):
        self.client = OpenAI(api_key=openai_api_key)
        self.voice = voice
        self.model = model

    def speak(self, text):
        # 녹음(STT)과 대칭으로 재생 시작/완료를 로그로 남긴다. flush=True는
        # ros2 launch가 자식 프로세스 stdout을 파이프로 연결할 때 print()가
        # 완전 버퍼링돼 지연되는 걸 막기 위함(stt.py와 동일한 이유).
        print(f"TTS 합성 중: '{text}'", flush=True)

        response = self.client.audio.speech.create(
            model=self.model,
            voice=self.voice,
            input=text,
            response_format="wav",
        )

        samplerate, data = wav.read(io.BytesIO(response.content))
        sd.play(data, samplerate)
        sd.wait()

        print("TTS 재생 완료", flush=True)

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile

class STT:
    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        # self.openai_api_key = openai_api_key
        self.duration = 5  # seconds
        self.samplerate = 16000  # Whisper는 16kHz를 선호


    def speech2text(self):
        # ros2 launch는 자식 프로세스 stdout을 파이프로 연결하는데, 그러면 파이썬
        # print()가 tty에 붙었을 때와 달리 완전 버퍼링돼서 바로 안 나오고 한참
        # 늦게(또는 프로세스 종료 시 몰아서) 찍힌다. 녹음/전송처럼 몇 초씩 걸리는
        # 단계는 그 사이에 진행 상황이 안 보이면 멈춘 것처럼 보이므로 flush=True로
        # 즉시 내보낸다.
        print("음성 녹음을 시작합니다. \n 5초 동안 말해주세요...", flush=True)
        audio = sd.rec(int(self.duration * self.samplerate), samplerate=self.samplerate, channels=1, dtype='int16')
        sd.wait()
        print("녹음 완료. Whisper에 전송 중...", flush=True)

        # 임시 WAV 파일 저장
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)

            # Whisper API 호출
            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f)

        print("STT 결과: ", transcript.text, flush=True)
        return transcript.text

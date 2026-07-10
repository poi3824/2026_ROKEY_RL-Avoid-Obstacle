import queue
import time

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile

from voice_interface.audio_level import normalized_rms


class STT:
    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        # self.openai_api_key = openai_api_key
        self.duration = 5  # seconds
        self.samplerate = 16000  # Whisper는 16kHz를 선호

    def speech2text(self, level_callback=None, stop_event=None):
        """5초간 녹음해 Whisper로 전사한다.

        2026-07-10: 이전엔 sd.rec()로 5초를 통째로 블로킹 녹음해서 녹음 도중의
        오디오 레벨을 알 방법이 없었다(HMI STT-TTS 탭의 오브를 실시간으로
        움직이려면 이게 필요함). sd.InputStream 콜백 방식으로 바꿔서, 매 블록
        (약 100ms)마다 level_callback(0.0~1.0)을 호출할 수 있게 한다.
        level_callback이 None이면(기존 호출부 호환) 그냥 조용히 녹음만 한다.

        stop_event(threading.Event)가 세팅되면 5초를 다 안 채우고 그 시점까지
        녹음된 것만으로 바로 전사한다 - HMI의 수동 녹음 버튼으로 "중지"를 누른
        경우에 쓴다.
        """
        # ros2 launch는 자식 프로세스 stdout을 파이프로 연결하는데, 그러면 파이썬
        # print()가 tty에 붙었을 때와 달리 완전 버퍼링돼서 바로 안 나오고 한참
        # 늦게(또는 프로세스 종료 시 몰아서) 찍힌다. 녹음/전송처럼 몇 초씩 걸리는
        # 단계는 그 사이에 진행 상황이 안 보이면 멈춘 것처럼 보이므로 flush=True로
        # 즉시 내보낸다.
        print("음성 녹음을 시작합니다. \n 5초 동안 말해주세요...", flush=True)

        block_q = queue.Queue()

        def _on_block(indata, frames, time_info, status):
            if status:
                print(f"[STT] sounddevice status: {status}", flush=True)
            block_q.put(indata.copy())

        blocks = []
        # blocksize=1600 @ 16kHz = 100ms - 레벨 콜백 주기이자 UI 반응 주기.
        with sd.InputStream(
            samplerate=self.samplerate, channels=1, dtype="int16",
            blocksize=1600, callback=_on_block,
        ):
            start = time.time()
            while time.time() - start < self.duration:
                if stop_event is not None and stop_event.is_set():
                    break
                try:
                    block = block_q.get(timeout=0.2)
                except queue.Empty:
                    continue
                blocks.append(block)
                if level_callback is not None:
                    level_callback(normalized_rms(block))

        audio = np.concatenate(blocks, axis=0) if blocks else np.zeros((0, 1), dtype="int16")
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

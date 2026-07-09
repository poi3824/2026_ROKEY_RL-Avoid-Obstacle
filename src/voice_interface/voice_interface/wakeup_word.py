import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

# 2026-07: cobot_ws/pick_and_place_voice에서 robot_ws/voice_interface로 이전.
# 원래 코드는 여기 PACKAGE_NAME이 "pick_and_place_voice"였는데, 그 옆의
# get_keyword_node.py는 .env를 "voice_processing"이라는 또 다른 패키지에서
# 읽어오고 있었음 (패키지 두 개에 리소스가 나뉘어 있던 상태). 이번 이전에서
# .env와 tflite 모델을 전부 이 패키지(voice_interface) 하나로 합침.
PACKAGE_NAME = "voice_interface"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

MODEL_NAME = "hello_rokey_8332_32.tflite"
MODEL_PATH = os.path.join(PACKAGE_PATH, f"resource/{MODEL_NAME}")

class WakeupWord:
    def __init__(self, buffer_size):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size

    def is_wakeup(self):
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name]
        # Wakeword 탐지
        if confidence > 0.3:
            print("Wakeword detected!", flush=True)
            return True
        return False

    def set_stream(self, stream):
        self.model = Model(wakeword_models=[MODEL_PATH])
        self.stream = stream

    def reset(self):
        """예측/오디오 피처 버퍼를 초기화한다.

        2026-07-09: 5초 말하고 뒤에 오탐하는거 리셋
        """
        if self.model is not None:
            self.model.reset()

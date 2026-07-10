"""int16 PCM 오디오 청크에서 UI 반응용 정규화 레벨(0.0~1.0)을 뽑는 작은 헬퍼.

wakeup_word.py(웨이크워드 대기 중)와 stt.py(명령 녹음 중) 둘 다 같은 방식으로
레벨을 계산해야 HMI 오브가 두 구간을 오갈 때 갑자기 튀지 않는다.
"""
import numpy as np

# int16 풀스케일(32768)을 그대로 분모로 쓰면 실제 발화에서도 값이 너무 작게
# 나온다(사람 말소리는 풀스케일을 거의 안 씀). 체감상 자연스러운 범위(약 이
# 값 근처를 "크게 말함"으로 취급)로 스케일링한다 - 정밀한 dBFS 계산이 아니라
# UI 반응성 튜닝용 근사치.
LOUD_REFERENCE_RMS = 4000.0


def normalized_rms(int16_chunk, scale=LOUD_REFERENCE_RMS):
    """int16 PCM 청크의 RMS를 0.0~1.0으로 정규화해서 반환한다 (1.0에서 클리핑)."""
    chunk = np.asarray(int16_chunk)
    if chunk.size == 0:
        return 0.0
    rms = float(np.sqrt(np.mean(np.square(chunk.astype(np.float32)))))
    return min(1.0, rms / scale)

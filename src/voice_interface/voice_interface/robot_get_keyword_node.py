# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import threading
import time

import pyaudio
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from voice_interface.MicController import MicController, MicConfig
from voice_interface.stt import STT
from voice_interface.tts import TTS
from voice_interface.wakeup_word import WakeupWord


# ============================================================
# Package Path & Environment Setting
# ============================================================

PACKAGE_NAME = "voice_interface"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")

load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

if openai_api_key is None:
    raise RuntimeError(f"OPENAI_API_KEY not found: {ENV_PATH}")


# ============================================================
# ROS 2 Interface Setting
# ============================================================

# 음성 STOP 명령은 brain_node를 거치지 않고 safety_monitor_node로 바로 전달한다.
# safety_monitor_node가 /voice/estop 토픽을 구독하여 실제 로봇 정지를 수행한다.
VOICE_ESTOP_TOPIC = "/voice/estop"

# brain_node가 로봇이 말할 문장을 이 토픽으로 발행한다.
#
# 통신 구조:
# brain_node
#     │  std_msgs/msg/String
#     ▼
# /tts/speak
#     │
#     ▼
# voice_interface(get_keyword_node)
#     │
#     ▼
# OpenAI TTS 음성 재생
TTS_TOPIC = "/tts/speak"
TTS_VOICE = "aria"

# STT 결과에 아래 단어가 포함되면 LLM 호출을 기다리지 않고 즉시 정지한다.
STOP_KEYWORDS = ("정지", "멈춰", "스톱", "중지", "그만")

# 웨이크워드 한 번 감지 후 음성 명령을 받을 최대 시간
MAX_SESSION_SEC = 8.0

# 객체 또는 목적지 일부만 인식된 상태를 유지하는 시간
PENDING_TIMEOUT_SEC = 15.0

# brain_node가 /get_keyword 서비스를 호출한 뒤 명령을 기다리는 최대 시간
SERVICE_WAIT_TIMEOUT_SEC = 30.0

ASK_OBJECT_MESSAGE = (
    "어떤 물체를 옮길까요? 빨간색, 파란색, 초록색 중에서 말씀해주세요."
)
ASK_TARGET_MESSAGE = (
    "어디로 옮길까요? 1번, 2번, 3번 위치 중에서 말씀해주세요."
)
ASK_SOURCE_MESSAGE = (
    "어디에 있는 물체인가요? 시작 위치, 1번, 2번, 3번 중에서 말씀해주세요."
)


class GetKeyword(Node):
    def __init__(self):
        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0,
            openai_api_key=openai_api_key,
        )

        prompt_content = """
당신은 사용자의 자연어 명령에서 이동해야 할 객체(Object), 출발지(Source Position),
목적지(Destination Position), 작업 후 복귀 위치(Return Position)를 추출하는 AI입니다.

<목표>
- 사용자의 문장에서 이동 대상 객체(Object), 출발지(Source Position),
  목적지(Destination Position)를 추출하세요.
- 작업 완료 후 로봇팔은 항상 대기 위치(home)로 복귀해야 합니다.
- 반드시 아래 리스트에 있는 이름만 사용하세요.
- 불명확한 항목은 UNKNOWN으로 출력하세요.

<객체 리스트>
- obj_A
- obj_B
- obj_C
- UNKNOWN

<YOLO 클래스 매핑>
- obj_B → class id 0
- obj_C → class id 1
- hand → class id 2
- obstacle → class id 3
- obj_A → class id 4
- 사용자의 명령에는 class id 번호를 출력하지 않습니다.
- 항상 obj_A/obj_B/obj_C 이름으로만 출력하세요.

<위치 리스트>
- home
- scan
- target1
- target2
- target3
- UNKNOWN

<위치 의미>
- home: 로봇팔의 대기 위치
- scan: 색깔 통을 처음으로 잡는 위치
- target1: 시약을 놓는 위치 1
- target2: 시약을 놓는 위치 2
- target3: 시약을 놓는 위치 3

<안전 규칙>
- 사용자가 "멈춰", "정지", "스톱", "중지", "그만"이라고 말하면
  STOP / STOP / STOP / STOP 으로 출력하세요.
- STOP은 완전 정지 명령입니다.
- 사용자가 "다시 시작해", "재개", "계속해", "다시 움직여", "동작해"처럼
  정지 상태를 풀고 다시 동작하라는 취지로 말하면
  RESUME / RESUME / RESUME / RESUME 으로 출력하세요.
- RESUME은 정지 해제 의도가 명확할 때만 출력하세요.

<월드맵 규칙>
- 사용자가 "월드맵 맵핑", "월드맵 매핑", "월드맵 스캔",
  "월드맵 업데이트"를 말하면
  WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP 으로 출력하세요.
- 안전 정지 명령이 포함된 경우에는 WORLD_MAP보다 STOP을 우선하세요.

<손 감지 무시 규칙>
- 사용자가 "손 아니야", "손 아님", "그거 손 아니야", "손 감지 무시해",
  "손 아니라고"처럼 말하면
  IGNORE_HAND / IGNORE_HAND / IGNORE_HAND / IGNORE_HAND 으로 출력하세요.
- 안전 정지 명령이 포함된 경우에는 IGNORE_HAND보다 STOP을 우선하세요.

<출력 형식>
객체 / 출발지 / 목적지 / 복귀위치

<규칙>
- 네 개 항목은 반드시 " / "로 구분합니다.
- 여러 객체가 있을 경우 같은 항목 안에서 공백으로 구분합니다.
- 출발지가 명시되지 않은 경우 기본값은 scan으로 간주합니다.
- 단, 사용자가 객체만 말한 경우 출발지는 UNKNOWN, 목적지는 UNKNOWN으로 출력합니다.
- 사용자가 목적지만 말한 경우 객체는 UNKNOWN, 출발지는 UNKNOWN으로 출력합니다.
- 목적지가 명시되지 않은 경우 목적지는 UNKNOWN으로 출력합니다.
- 작업 완료 후 복귀 위치는 항상 home입니다.
- 설명이나 추가 문장은 절대 출력하지 않습니다.
- 출력은 반드시 한 줄만 작성합니다.

<객체 매핑>
- "빨간색 통", "빨간거", "빨간색", "빨간 시약", "빨강" → obj_A
- "파란색 통", "파란거", "파란색", "파란 시약", "파랑" → obj_B
- "초록색 통", "초록거", "초록색", "초록 시약", "초록" → obj_C

<위치 매핑>
- "대기 위치", "대기 지점", "홈", "home" → home
- "처음 위치", "집는 위치", "시작 위치", "시작지점", "스캔 위치",
  "출발 위치", "0번", "0번 위치" → scan
- "1번", "1번 위치", "타겟1", "타켓1" → target1
- "2번", "2번 위치", "타겟2", "타켓2" → target2
- "3번", "3번 위치", "타겟3", "타켓3" → target3

<예시>
입력:
빨간색 통을 1번 위치로 옮겨
출력:
obj_A / scan / target1 / home

입력:
빨간색 통
출력:
obj_A / UNKNOWN / UNKNOWN / home

입력:
2번으로
출력:
UNKNOWN / UNKNOWN / target2 / home

입력:
월드맵 스캔 시작
출력:
WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP

입력:
손 아니야
출력:
IGNORE_HAND / IGNORE_HAND / IGNORE_HAND / IGNORE_HAND

입력:
멈춰
출력:
STOP / STOP / STOP / STOP

입력:
다시 시작해
출력:
RESUME / RESUME / RESUME / RESUME

<사용자 입력>
"{user_input}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template=prompt_content,
        )
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)

        super().__init__("get_keyword_node")

        # ========================================================
        # Microphone / Wakeword
        # ========================================================

        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )

        self.mic_controller = MicController(config=mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # ========================================================
        # Publisher / Client / Subscriber / Service
        # ========================================================

        # STOP 발행:
        # voice_interface -> /voice/estop -> safety_monitor_node
        self.estop_pub = self.create_publisher(
            Bool,
            VOICE_ESTOP_TOPIC,
            10,
        )

        # RESUME과 손 감지 무시는 brain_node를 거치지 않고
        # voice_interface가 safety_monitor 서비스에 직접 요청한다.
        self.safety_reset_client = self.create_client(
            Trigger,
            "/safety/reset",
        )
        self.ignore_hand_client = self.create_client(
            Trigger,
            "/safety/ignore_hand",
        )

        # brain_node가 /tts/speak 토픽으로 발행한 문장을 받아 음성으로 재생한다.
        #
        # 예:
        # self.tts_pub = self.create_publisher(String, "/tts/speak", 10)
        # self.tts_pub.publish(String(data="작업을 시작합니다."))
        self.create_subscription(
            String,
            TTS_TOPIC,
            self._on_speak,
            10,
        )

        # brain_node가 이 서비스를 호출하면 인식된 최신 명령을 응답으로 반환한다.
        #
        # brain_node(client)
        #     │ Trigger request
        #     ▼
        # /get_keyword
        #     │ Trigger response.message
        #     ▼
        # "obj_A / scan / target1 / home"
        self.get_keyword_srv = self.create_service(
            Trigger,
            "/get_keyword",
            self.get_keyword,
        )

        # ========================================================
        # Shared Command State
        # ========================================================

        # 백그라운드 음성 인식 스레드가 명령을 저장하고,
        # /get_keyword 서비스 콜백이 해당 명령을 꺼내 brain_node에 응답한다.
        self._lock = threading.Lock()
        self._latest_command = None
        self._command_ready = threading.Event()

        # 슬롯 필링 상태
        self._pending_slots = self._new_pending_slots()
        self._pending_time = 0.0

        # STT 녹음과 TTS 재생이 동시에 오디오 장치를 사용하지 않도록 보호
        self._audio_lock = threading.Lock()
        self.tts = TTS(openai_api_key, voice=TTS_VOICE)
        self._speaking = threading.Event()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for brain_node's /get_keyword request...")

        # /get_keyword 서비스 호출 여부와 무관하게 항상 음성을 듣는다.
        self._listen_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True,
        )
        self._listen_thread.start()

    def _new_pending_slots(self):
        return {
            "obj": None,
            "source": None,
            "target": None,
            "return_pos": "home",
        }

    def _listen_loop(self):
        """
        웨이크워드 → STT → LLM 명령 파싱을 백그라운드에서 반복한다.

        일반 이동 명령과 WORLD_MAP 명령:
        - self._latest_command에 저장
        - brain_node가 /get_keyword 서비스를 호출하면 response.message로 전달

        STOP:
        - brain_node를 거치지 않고 /voice/estop 토픽으로 즉시 발행

        RESUME:
        - brain_node를 거치지 않고 /safety/reset 서비스 직접 호출

        IGNORE_HAND:
        - brain_node를 거치지 않고 /safety/ignore_hand 서비스 직접 호출
        """
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)

        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return

        while rclpy.ok():
            # TTS 재생 중에는 로봇의 목소리를 웨이크워드로 오인식하지 않도록 정지
            if self._speaking.is_set():
                time.sleep(0.05)
                continue

            if not self.wakeup_word.is_wakeup():
                continue

            self.get_logger().info("헬로우 로키 감지 - 녹음 시작")
            session_start = time.time()

            try:
                while (
                    rclpy.ok()
                    and (time.time() - session_start) < MAX_SESSION_SEC
                ):
                    if self._speaking.is_set():
                        time.sleep(0.05)
                        continue

                    with self._audio_lock:
                        output_message = self.stt.speech2text()

                    self._flush_mic_stream()
                    self.get_logger().info(
                        f"STT 인식 결과: '{output_message}'"
                    )

                    if output_message is None or len(output_message.strip()) < 2:
                        self.get_logger().warn(
                            "STT 결과가 비었거나 너무 짧음 — 이번 라운드 무시"
                        )
                        continue

                    # 안전 정지는 LLM 왕복 전에 로컬 문자열로 즉시 처리한다.
                    if any(word in output_message for word in STOP_KEYWORDS):
                        self.get_logger().warn(
                            f"정지 키워드 감지(로컬): "
                            f"'{output_message}' -> 응급정지 호출"
                        )
                        self._clear_pending_slots()
                        self._call_emergency_stop()

                        # brain_node가 이후 /get_keyword를 호출했을 때도
                        # STOP 상태를 알 수 있도록 최신 명령에 저장한다.
                        self._set_latest_command(
                            "STOP / STOP / STOP / STOP"
                        )
                        break

                    obj, source, target, return_pos = self.extract_keyword(
                        output_message
                    )

                    if obj is None:
                        self.get_logger().error(
                            "Failed to extract keyword from LLM response"
                        )
                        continue

                    if "STOP" in obj:
                        self.get_logger().warn(
                            f"정지 키워드 감지(LLM): "
                            f"STT='{output_message}' -> 응급정지 호출"
                        )
                        self._clear_pending_slots()
                        self._call_emergency_stop()
                        self._set_latest_command(
                            "STOP / STOP / STOP / STOP"
                        )
                        break

                    if "RESUME" in obj:
                        self.get_logger().warn(
                            f"재개 명령 감지(LLM): STT='{output_message}'"
                        )
                        self._clear_pending_slots()
                        self._call_safety_reset()
                        self._speak_locally("다시 시작합니다")
                        break

                    if "IGNORE_HAND" in obj:
                        self.get_logger().warn(
                            f"손 감지 무시 명령(LLM): STT='{output_message}'"
                        )
                        self._clear_pending_slots()
                        self._call_ignore_hand()
                        self._speak_locally("손 아닌 걸로 하겠습니다")
                        break

                    if "WORLD_MAP" in obj:
                        self._clear_pending_slots()
                        self._set_latest_command(
                            "WORLD_MAP / WORLD_MAP / "
                            "WORLD_MAP / WORLD_MAP"
                        )
                        break

                    # 불완전한 명령이면 기존 슬롯과 합쳐 완성한다.
                    completed_command = self._update_slots_and_build_command(
                        obj,
                        source,
                        target,
                        return_pos,
                    )

                    if completed_command is None:
                        missing_message = self._get_missing_slot_message()
                        self.get_logger().warn(
                            f"명령 정보 부족 -> 질문: {missing_message}"
                        )

                        # 같은 노드 내부에서 /tts/speak로 발행하여
                        # _on_speak 콜백이 재생하도록 한다.
                        self._speak_guide(missing_message)
                        continue

                    self.get_logger().warn(
                        f"Detected command: {completed_command}"
                    )

                    self._clear_pending_slots()

                    # 인식된 명령을 저장한다.
                    # brain_node가 /get_keyword 서비스를 호출하면 이 값을 받는다.
                    self._set_latest_command(completed_command)
                    break

            finally:
                self._flush_mic_stream()

                try:
                    self.wakeup_word.reset()
                except Exception as e:
                    self.get_logger().warn(
                        f"Wakeword reset 실패(무시): {e}"
                    )

    def _update_slots_and_build_command(
        self,
        obj,
        source,
        target,
        return_pos,
    ):
        now = time.time()

        if (
            self._has_pending_slots()
            and now - self._pending_time > PENDING_TIMEOUT_SEC
        ):
            self.get_logger().warn(
                "보류 명령 시간이 초과되어 초기화합니다."
            )
            self._clear_pending_slots()

        self._pending_time = now

        obj_value = self._first_valid(obj)
        source_value = self._first_valid(source)
        target_value = self._first_valid(target)
        return_value = self._first_valid(return_pos)

        if obj_value is not None:
            self._pending_slots["obj"] = obj_value

        if source_value is not None:
            self._pending_slots["source"] = source_value

        if target_value is not None:
            self._pending_slots["target"] = target_value

        if return_value is not None:
            self._pending_slots["return_pos"] = return_value

        # 객체를 인식했는데 출발지가 없다면 기본 출발지는 scan
        if (
            self._pending_slots["source"] is None
            and self._pending_slots["obj"] is not None
        ):
            self._pending_slots["source"] = "scan"

        if not self._is_slots_complete():
            return None

        return (
            f"{self._pending_slots['obj']} / "
            f"{self._pending_slots['source']} / "
            f"{self._pending_slots['target']} / "
            f"{self._pending_slots['return_pos']}"
        )

    def _first_valid(self, values):
        for value in values:
            if value not in (
                "UNKNOWN",
                "STOP",
                "RESUME",
                "WORLD_MAP",
                "IGNORE_HAND",
            ):
                return value

        return None

    def _has_pending_slots(self):
        return (
            self._pending_slots["obj"] is not None
            or self._pending_slots["source"] is not None
            or self._pending_slots["target"] is not None
        )

    def _is_slots_complete(self):
        return (
            self._pending_slots["obj"] is not None
            and self._pending_slots["source"] is not None
            and self._pending_slots["target"] is not None
            and self._pending_slots["return_pos"] is not None
        )

    def _get_missing_slot_message(self):
        if self._pending_slots["obj"] is None:
            return ASK_OBJECT_MESSAGE

        if self._pending_slots["target"] is None:
            return ASK_TARGET_MESSAGE

        if self._pending_slots["source"] is None:
            return ASK_SOURCE_MESSAGE

        return "명령을 완성하지 못했습니다. 다시 말씀해주세요."

    def _clear_pending_slots(self):
        self._pending_slots = self._new_pending_slots()
        self._pending_time = 0.0

    def _set_latest_command(self, keyword_str):
        """
        백그라운드 음성 인식 스레드가 완성한 명령을 저장한다.

        이 함수 자체가 brain_node로 토픽을 발행하는 것은 아니다.
        저장된 명령은 brain_node가 /get_keyword 서비스를 호출했을 때
        get_keyword()의 response.message로 반환된다.
        """
        with self._lock:
            self._latest_command = keyword_str

        self._command_ready.set()

    def _speak_guide(self, text):
        """
        슬롯이 부족할 때 안내 문장을 /tts/speak 토픽으로 발행한다.

        이 노드는 동시에 /tts/speak를 구독하고 있으므로
        발행한 문장은 _on_speak()에서 받아 실제 음성으로 재생된다.
        """
        guide_msg = String()
        guide_msg.data = text

        # 안내용 publisher는 필요할 때 한 번 생성하는 것보다
        # __init__에서 생성하는 편이 일반적이므로 아래 속성을 확인한다.
        if not hasattr(self, "tts_pub"):
            self.tts_pub = self.create_publisher(
                String,
                TTS_TOPIC,
                10,
            )

        self.tts_pub.publish(guide_msg)

    def _flush_mic_stream(self):
        try:
            stream = self.mic_controller.stream

            if stream is None:
                return

            available = stream.get_read_available()

            if available > 0:
                stream.read(
                    available,
                    exception_on_overflow=False,
                )

        except Exception as e:
            self.get_logger().warn(
                f"마이크 버퍼 플러시 실패(무시): {e}"
            )

    def _call_emergency_stop(self):
        """
        /voice/estop 토픽에 True를 발행한다.

        실제 로봇 정지는 safety_monitor_node가 담당한다.
        """
        self.estop_pub.publish(Bool(data=True))

    def _speak_locally(self, text):
        text = text.strip()

        if not text:
            return

        self._speaking.set()

        try:
            with self._audio_lock:
                self.tts.speak(text)

        except Exception as e:
            self.get_logger().error(
                f"TTS 재생 실패(무시): {e}"
            )

        finally:
            self._flush_mic_stream()
            self._speaking.clear()

    def _on_speak(self, msg):
        """
        brain_node 또는 이 노드가 /tts/speak로 발행한 텍스트를 재생한다.
        """
        self._speak_locally(msg.data)

    def _call_safety_reset(self):
        if not self.safety_reset_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                "/safety/reset 서비스 없음 — RESUME 처리 실패"
            )
            return

        future = self.safety_reset_client.call_async(
            Trigger.Request()
        )
        start = time.time()

        while (
            not future.done()
            and time.time() - start < 2.0
        ):
            time.sleep(0.01)

        if not future.done():
            self.get_logger().error(
                "/safety/reset 서비스 응답 시간 초과"
            )
            return

        try:
            result = future.result()

            if result is not None and not result.success:
                self.get_logger().error(
                    f"/safety/reset 실패: {result.message}"
                )

        except Exception as e:
            self.get_logger().error(
                f"/safety/reset 호출 오류: {e}"
            )

    def _call_ignore_hand(self):
        if not self.ignore_hand_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                "/safety/ignore_hand 서비스 없음 — 처리 실패"
            )
            return

        future = self.ignore_hand_client.call_async(
            Trigger.Request()
        )
        start = time.time()

        while (
            not future.done()
            and time.time() - start < 2.0
        ):
            time.sleep(0.01)

        if not future.done():
            self.get_logger().error(
                "/safety/ignore_hand 서비스 응답 시간 초과"
            )
            return

        try:
            result = future.result()

            if result is not None and not result.success:
                self.get_logger().error(
                    f"/safety/ignore_hand 실패: {result.message}"
                )

        except Exception as e:
            self.get_logger().error(
                f"/safety/ignore_hand 호출 오류: {e}"
            )

    def extract_keyword(self, output_message):
        try:
            response = self.lang_chain.invoke(
                {"user_input": output_message}
            )
            result = response.content.strip()

        except Exception as e:
            self.get_logger().error(f"LLM 호출 실패: {e}")
            return None, None, None, None

        print(f"llm raw response: {result}")

        parts = [
            part.strip()
            for part in result.split("/")
        ]

        if len(parts) != 4:
            self.get_logger().error(
                f"Invalid LLM format: {result}"
            )
            return None, None, None, None

        obj, source, target, return_pos = parts

        obj = obj.split()
        source = source.split()
        target = target.split()
        return_pos = return_pos.split()

        print(f"object: {obj}")
        print(f"source: {source}")
        print(f"target: {target}")
        print(f"return_pos: {return_pos}")

        return obj, source, target, return_pos

    def get_keyword(self, request, response):
        """
        brain_node가 /get_keyword 서비스를 호출하면 실행되는 콜백이다.

        백그라운드 _listen_loop가 명령을 인식할 때까지 기다린 뒤,
        response.message에 명령 문자열을 담아 brain_node에 반환한다.
        """
        command_received = self._command_ready.wait(
            timeout=SERVICE_WAIT_TIMEOUT_SEC
        )

        if not command_received:
            response.success = False
            response.message = "TIMEOUT"
            return response

        with self._lock:
            keyword_str = self._latest_command
            self._latest_command = None

        self._command_ready.clear()

        if keyword_str is None:
            response.success = False
            response.message = "NO_COMMAND"
            return response

        response.success = True
        response.message = keyword_str

        self.get_logger().info(
            f"brain_node에 명령 응답: {keyword_str}"
        )

        return response


def main():
    rclpy.init()
    node = GetKeyword()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
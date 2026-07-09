# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import threading
import time

import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String

from voice_interface.MicController import MicController, MicConfig
from voice_interface.wakeup_word import WakeupWord
from voice_interface.stt import STT
from voice_interface.tts import TTS


PACKAGE_NAME = "voice_interface"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")

load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

if openai_api_key is None:
    raise RuntimeError(f"OPENAI_API_KEY not found: {ENV_PATH}")


VOICE_ESTOP_TOPIC = "/voice/estop"
TTS_TOPIC = "/tts/speak"
TTS_VOICE = "alloy"

STOP_KEYWORDS = ("정지", "멈춰", "스톱", "중지", "그만")

MAX_SESSION_SEC = 8.0
PENDING_TIMEOUT_SEC = 15.0

ASK_OBJECT_MESSAGE = "어떤 물체를 옮길까요? 빨간색, 파란색, 초록색 중에서 말씀해주세요."
ASK_TARGET_MESSAGE = "어디로 옮길까요? 1번, 2번, 3번 위치 중에서 말씀해주세요."
ASK_SOURCE_MESSAGE = "어디에 있는 물체인가요? 시작 위치, 1번, 2번, 3번 중에서 말씀해주세요."


class GetKeyword(Node):
    def __init__(self):
        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0,
            openai_api_key=openai_api_key,
        )

        prompt_content = """
당신은 사용자의 자연어 명령에서 이동해야 할 객체(Object), 출발지(Source Position), 목적지(Destination Position), 작업 후 복귀 위치(Return Position)를 추출하는 AI입니다.

<목표>
- 사용자의 문장에서 이동 대상 객체(Object), 출발지(Source Position), 목적지(Destination Position)를 추출하세요.
- 작업 완료 후 로봇팔은 항상 대기 위치(home)로 복귀해야 합니다.
- 반드시 아래 리스트에 있는 이름만 사용하세요.
- 불명확한 항목은 UNKNOWN으로 출력하세요.

<객체 리스트>
- obj_A
- obj_B
- obj_C
- UNKNOWN

<위치 리스트>
- home
- scan
- target1
- target2
- target3
- UNKNOWN

<위치 의미>
- home: 로봇팔의 대기 위치
- scan: 색깔 통(시약)을 처음으로 잡는 위치
- target1: 시약을 놓는 위치 1
- target2: 시약을 놓는 위치 2
- target3: 시약을 놓는 위치 3

<이동 규칙>
- scan, target1, target2, target3 사이의 이동은 모두 가능합니다.
- home은 작업 시작 전/작업 완료 후/정지 명령 시 로봇팔이 이동하는 대기 위치입니다.
- 일반적인 물체 이동 명령에서는 home을 출발지나 목적지로 사용하지 않습니다.
- 작업이 완료되면 return 위치는 항상 home입니다.

<안전 규칙>
- 사용자가 "멈춰", "정지", "스톱", "중지", "그만"이라고 말하면 즉시 안전 정지 명령으로 판단합니다.
- 안전 정지 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 STOP으로 출력하세요.
- 입력에 안전 정지 단어가 포함되면 다른 모든 명령보다 STOP을 최우선으로 출력합니다.

<재개 규칙>
- 사용자가 "다시 시작해", "재개", "계속해", "다시 움직여", "동작해"처럼 정지 상태를 풀고 다시 동작하라는 취지로 말하면 재개 명령으로 판단합니다.
- 재개 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 RESUME으로 출력하세요.
- RESUME은 정지 해제 의도가 명확할 때만 출력합니다.

<월드맵 규칙>
- 사용자가 "월드맵 맵핑", "월드맵 매핑", "월드맵 스캔", "월드맵 업데이트"를 말하면 월드맵 작업 명령으로 판단합니다.
- 월드맵 작업 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 WORLD_MAP으로 출력하세요.
- 안전 정지 명령이 포함된 경우에는 월드맵 명령보다 STOP을 우선합니다.

<출력 형식>
객체 / 출발지 / 목적지 / 복귀위치

<규칙>
- 네 개 항목은 반드시 " / "로 구분합니다.
- 여러 객체가 있을 경우 같은 항목 안에서 공백으로 구분합니다.
- 객체 수와 출발지 수와 목적지 수는 서로 대응되도록 작성합니다.
- 출발지가 명시되지 않은 경우 기본값은 scan으로 간주합니다.
- 단, 사용자가 목적지만 말한 경우 출발지는 UNKNOWN으로 출력합니다.
- 목적지가 명시되지 않은 경우 목적지는 UNKNOWN으로 출력합니다.
- 사용자가 객체만 말한 경우 출발지는 UNKNOWN, 목적지는 UNKNOWN으로 출력합니다.
- 사용자가 출발지만 말한 경우 객체는 UNKNOWN, 목적지는 UNKNOWN으로 출력합니다.
- 사용자가 목적지만 말한 경우 객체는 UNKNOWN, 출발지는 UNKNOWN으로 출력합니다.
- 작업 완료 후 복귀 위치는 항상 home입니다.
- 설명이나 추가 문장은 절대 출력하지 않습니다.
- 출력은 반드시 한 줄만 작성합니다.
- "출력:", "결과:", "설명:" 등의 문구를 절대 포함하지 않습니다.

<객체 매핑>
- "빨간색 통", "빨간거", "빨간색", "빨간 시약", "빨강" → obj_A
- "파란색 통", "파란거", "파란색", "파란 시약", "파랑" → obj_B
- "초록색 통", "초록거", "초록색", "초록 시약", "초록" → obj_C

<위치 매핑>
- "대기 위치", "대기 지점", "홈", "home" → home
- "처음 위치", "집는 위치", "시작 위치", "시작지점", "스캔 위치", "출발 위치", "0번", "0번 위치" → scan
- "1번", "1번 위치", "타겟1", "타켓1" → target1
- "2번", "2번 위치", "타겟2", "타켓2" → target2
- "3번", "3번 위치", "타겟3", "타켓3" → target3

<예시>
입력:
빨간색 통을 1번 위치로 옮겨
출력:
obj_A / scan / target1 / home

입력:
파란색 통을 2번으로 이동시켜
출력:
obj_B / scan / target2 / home

입력:
초록색 통을 3번 위치로 옮겨
출력:
obj_C / scan / target3 / home

입력:
1번 위치에 있는 빨간색 통을 2번 위치로 옮겨
출력:
obj_A / target1 / target2 / home

입력:
2번 위치의 파란색 통을 3번으로 옮겨
출력:
obj_B / target2 / target3 / home

입력:
3번에 있는 빨간색 통을 시작지점으로 가져와
출력:
obj_A / target3 / scan / home

입력:
빨간색 통은 1번으로, 파란색 통은 2번으로 옮겨
출력:
obj_A obj_B / scan scan / target1 target2 / home

입력:
빨간색 통을 이동시켜
출력:
obj_A / scan / UNKNOWN / home

입력:
빨간색 통
출력:
obj_A / UNKNOWN / UNKNOWN / home

입력:
파란색
출력:
obj_B / UNKNOWN / UNKNOWN / home

입력:
1번으로
출력:
UNKNOWN / UNKNOWN / target1 / home

입력:
2번 위치로
출력:
UNKNOWN / UNKNOWN / target2 / home

입력:
3번으로
출력:
UNKNOWN / UNKNOWN / target3 / home

입력:
1번에 있는 거
출력:
UNKNOWN / target1 / UNKNOWN / home

입력:
2번 위치에 있는 물체
출력:
UNKNOWN / target2 / UNKNOWN / home

입력:
시작 위치에 있는 거
출력:
UNKNOWN / scan / UNKNOWN / home

입력:
월드맵 스캔 시작
출력:
WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP

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

        self.estop_pub = self.create_publisher(Bool, VOICE_ESTOP_TOPIC, 10)
        self.tts_pub = self.create_publisher(String, TTS_TOPIC, 10)

        self._lock = threading.Lock()
        self._latest_command = None
        self._command_ready = threading.Event()

        self._pending_slots = self._new_pending_slots()
        self._pending_time = 0.0

        self.tts = TTS(openai_api_key, voice=TTS_VOICE)
        self._speaking = threading.Event()
        self.create_subscription(String, TTS_TOPIC, self._on_speak, 10)

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")

        self.get_keyword_srv = self.create_service(
            Trigger,
            "get_keyword",
            self.get_keyword,
        )

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
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return

        while rclpy.ok():
            if self._speaking.is_set():
                time.sleep(0.05)
                continue

            if not self.wakeup_word.is_wakeup():
                continue

            self.get_logger().info("헬로우 로키 감지 - 녹음 시작")

            session_start = time.time()

            while rclpy.ok() and (time.time() - session_start) < MAX_SESSION_SEC:
                output_message = self.stt.speech2text()
                self._flush_mic_stream()

                self.get_logger().info(f"STT 인식 결과: '{output_message}'")

                if len(output_message.strip()) < 2:
                    self.get_logger().warn("STT 결과가 비었거나 너무 짧음 — 이번 라운드 무시")
                    continue

                if any(word in output_message for word in STOP_KEYWORDS):
                    self.get_logger().warn(
                        f"정지 키워드 감지(로컬): '{output_message}' -> 응급정지 호출"
                    )
                    self._clear_pending_slots()
                    self._call_emergency_stop()
                    continue

                obj, source, target, return_pos = self.extract_keyword(output_message)

                if obj is None:
                    self.get_logger().error("Failed to extract keyword from LLM response")
                    continue

                if "STOP" in obj:
                    self.get_logger().warn(
                        f"정지 키워드 감지(LLM): STT='{output_message}' -> 응급정지 호출"
                    )
                    self._clear_pending_slots()
                    self._call_emergency_stop()
                    continue

                if "WORLD_MAP" in obj:
                    self._clear_pending_slots()
                    self._set_latest_command("WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP")
                    continue

                if "RESUME" in obj:
                    self._clear_pending_slots()
                    self._set_latest_command("RESUME / RESUME / RESUME / RESUME")
                    continue

                completed_command = self._update_slots_and_build_command(
                    obj,
                    source,
                    target,
                    return_pos,
                )

                if completed_command is None:
                    missing_message = self._get_missing_slot_message()
                    self.get_logger().warn(f"명령 정보 부족 -> 질문: {missing_message}")
                    self._speak_guide(missing_message)
                    continue

                self.get_logger().warn(f"Detected command: {completed_command}")

                self._clear_pending_slots()
                self._set_latest_command(completed_command)

            self._flush_mic_stream()

    def _update_slots_and_build_command(self, obj, source, target, return_pos):
        now = time.time()

        if self._has_pending_slots() and now - self._pending_time > PENDING_TIMEOUT_SEC:
            self.get_logger().warn("보류 명령 시간이 초과되어 초기화합니다.")
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

        if self._pending_slots["source"] is None and self._pending_slots["obj"] is not None:
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
            if value not in ("UNKNOWN", "STOP", "RESUME", "WORLD_MAP"):
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
        with self._lock:
            self._latest_command = keyword_str
        self._command_ready.set()

    def _speak_guide(self, text):
        self.tts_pub.publish(String(data=text))

    def _flush_mic_stream(self):
        try:
            stream = self.mic_controller.stream
            available = stream.get_read_available()
            if available > 0:
                stream.read(available, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().warn(f"마이크 버퍼 플러시 실패(무시): {e}")

    def _call_emergency_stop(self):
        self.estop_pub.publish(Bool(data=True))

    def _on_speak(self, msg):
        text = msg.data.strip()

        if not text:
            return

        self._speaking.set()

        try:
            self.tts.speak(text)
        except Exception as e:
            self.get_logger().error(f"TTS 재생 실패(무시): {e}")
        finally:
            self._flush_mic_stream()
            self._speaking.clear()

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content.strip()

        print(f"llm raw response: {result}")

        parts = [part.strip() for part in result.split("/")]

        if len(parts) != 4:
            self.get_logger().error(f"Invalid LLM format: {result}")
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
        self._command_ready.wait()

        with self._lock:
            keyword_str = self._latest_command
            self._latest_command = None

        self._command_ready.clear()

        response.success = True
        response.message = keyword_str
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
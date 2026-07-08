# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import threading

import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate  # d2 이거를 langchain_core로 바꿈
# from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from voice_interface.MicController import MicController, MicConfig

from voice_interface.wakeup_word import WakeupWord
from voice_interface.stt import STT

############ Package Path & Environment Setting ############

#----------------------------------------------------------------
# current_dir = os.getcwd()
# package_path = get_package_share_directory("pick_and_place_voice")

# env_path = "/home/rokey/cobot_ws/src/cobot2_ws/pick_and_place_voice/resource/.env"
# load_dotenv(dotenv_path=env_path)
# is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
# openai_api_key = os.getenv("OPENAI_API_KEY")
#-----------------------------------------------------------------

PACKAGE_NAME = "voice_interface"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

# 2026-07-06: robot_action_node가 pick/place로 바쁜 동안에는 get_keyword 서비스가
# 호출되지 않아서, 그동안 웨이크워드/STT가 전혀 안 돌고 있었다 (음성으로 "정지"를
# 말해도 아무도 듣고 있지 않은 상태였음). 그래서 듣는 루프를 서비스 콜백 밖으로
# 빼서 항상 도는 백그라운드 스레드로 만들고, "정지"류는 로봇이 뭘 하고 있든
# robot_action_node를 거치지 않고 바로 emergency_stop 서비스를 호출하게 했다.
#
# 2026-07-07: 개편 후 음성 '정지'는 safety_monitor_node로 넘긴다. 이전엔 STT가
# /dsr01/emergency_stop 서비스를 직접 호출했지만, 이제는 안전 판단을 safety_monitor가
# 전담하므로 STT는 /voice/estop 토픽만 쏘고 빠진다. safety_monitor가 이걸 받아
# 로봇 move_stop을 직접 호출하고 /safety/state를 ESTOP으로 방송한다.
VOICE_ESTOP_TOPIC = "/voice/estop"

# STT 텍스트에 이 단어가 있으면 LLM 왕복을 기다리지 않고 바로 응급정지를 호출한다.
STOP_KEYWORDS = ("정지", "멈춰", "스톱", "중지", "그만")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
                    당신은 사용자의 자연어 명령에서 이동해야 할 객체(Object), 출발지(Source Position), 목적지(Destination Position), 작업 후 복귀 위치(Return Position)를 추출하는 AI입니다.

<목표>
- 사용자의 문장에서 이동 대상 객체(Object), 출발지(Source Position), 목적지(Destination Position)를 추출하세요.
- 작업 완료 후 로봇팔은 항상 대기 위치(home)로 복귀해야 합니다.
- 반드시 아래 리스트에 있는 이름만 사용하세요.

<객체 리스트>
- obj_A
- obj_B
- obj_C

<위치 리스트>
- home
- scan
- target1
- target2
- target3

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

<안전 규칙1>
- 사용자가 "멈춰", "정지", "스톱", "중지", "그만"이라고 말하면 즉시 안전 정지 명령으로 판단합니다.
- 안전 정지 명령이면 객체, 출발지, 목적지는 모두 STOP으로 출력하고, 복귀 위치도 STOP으로 출력하세요.
- 안전 정지 명령 출력은 반드시 아래 형식을 따르세요.
- STOP은 완전 정지 명령입니다. 
- 로봇의 작동을 완전히 정지시킵니다.
- 다시 시작해 달라는 명령이 있을 때까지 로봇은 어떠한 동작도 수행하지 않습니다.

<출력 형식>
- 반드시 아래 형식으로만 출력하세요.
객체 / 출발지 / 목적지 / 복귀위치    
STOP / STOP / STOP / STOP

반드시 아래 형식으로만 출력하세요.

객체 / 출발지 / 목적지 / 복귀위치

예:
obj_A / scan / target1 / home

<규칙>
- 네 개 항목은 반드시 " / "로 구분합니다.
- 여러 객체가 있을 경우 같은 항목 안에서 공백으로 구분합니다.
- 객체 수와 출발지 수와 목적지 수는 서로 대응되도록 작성합니다.
- 출발지가 명시되지 않은 경우 기본값은 scan으로 간주합니다.
- 목적지가 명시되지 않은 경우 목적지는 UNKNOWN으로 출력합니다.
- 작업 완료 후 복귀 위치는 항상 home입니다.
- 설명이나 추가 문장은 절대 출력하지 않습니다.

<객체 매핑>
- "빨간색 통", "빨간거", "빨간색", "빨간 시약" → obj_A
- "파란색 통", "파란거", "파란색", "파란 시약" → obj_B
- "초록색 통", "초록거", "초록색", "초록 시약" → obj_C

<위치 매핑>
- "대기 위치", "대기 지점", "홈", "home" → home
- "처음 위치", "집는 위치", "시작 위치", "스캔 위치", "출발 위치",→ scan
- "1번", "1번 위치", "타켓1" → target1
- "2번", "2번 위치", "타켓2" → target2
- "3번", "3번 위치", "타켓3" → target3

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
멈춰

출력:
STOP / STOP / STOP / STOP

입력:
그만해

출력:
STOP / STOP / STOP / STOP

입력:
정지

출력:
STOP / STOP / STOP / STOP

<사용자 입력>
"{user_input}"     
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        # self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
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
        # self.ai_processor = AIProcessor()

        self.estop_pub = self.create_publisher(Bool, VOICE_ESTOP_TOPIC, 10)

        # 2026-07-06: 백그라운드 리스닝 스레드(_listen_loop)와 get_keyword() 서비스
        # 콜백이 공유하는 상태. _listen_loop가 파싱한 최신 명령을 여기 넣어두면
        # get_keyword()가 그걸 꺼내서 응답으로 돌려준다.
        self._lock = threading.Lock()
        self._latest_command = None
        self._command_ready = threading.Event()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )

        # 2026-07-06: get_keyword 서비스 호출 여부와 무관하게 항상 듣는다.
        # daemon=True라서 노드가 죽으면 스레드도 같이 정리된다.
        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def _listen_loop(self):
        """2026-07-06: 웨이크워드 → 녹음 → STT를 무한 반복한다.

        get_keyword() 호출 여부와 무관하게 항상 돈다. STOP류가 감지되면
        robot_action_node를 거치지 않고 여기서 바로 emergency_stop을 호출하고,
        일반 명령은 self._latest_command에 저장해 get_keyword()가 꺼내가게 한다.
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
            while not self.wakeup_word.is_wakeup():
                pass

            # STT --> Keyword Extract --> Embedding
            output_message = self.stt.speech2text()
            self.get_logger().info(f"STT 인식 결과: '{output_message}'")

            # Whisper는 무음/잡음 구간에서 빈 문자열이나 아주 짧은 환청을 자주 낸다.
            # 그대로 LLM에 넣으면 UNKNOWN이나 엉뚱한 STOP으로 파싱돼 오작동(특히
            # 의도치 않은 응급정지)하므로 여기서 먼저 걸러낸다.
            if len(output_message.strip()) < 2:
                self.get_logger().warn("STT 결과가 비었/너무 짧음 — 무시")
                continue

            if any(word in output_message for word in STOP_KEYWORDS):
                self.get_logger().warn(f"정지 키워드 감지(로컬): '{output_message}' -> 응급정지 호출")
                self._call_emergency_stop()
                continue

            obj, source, target, return_pos = self.extract_keyword(output_message)

            # extract_keyword가 파싱에 실패해 None을 반환한 경우 방어 처리
            # (원래 코드는 이 경우에도 " ".join(keyword)를 호출해 TypeError가 발생했음)
            if obj is None:
                self.get_logger().error("Failed to extract keyword from LLM response")
                continue

            if "STOP" in obj:  # LLM이 STOP으로 파싱한 경우에 대한 2차 방어
                self.get_logger().warn(
                    f"정지 키워드 감지(LLM): STT='{output_message}' -> 응급정지 호출"
                )
                self._call_emergency_stop()
                continue

            # obj, source, target, return_pos는 각각 리스트이므로
            # " ".join(keyword)처럼 튜플을 바로 join하면
            # 각 원소가 str이 아닌 list라 TypeError가 발생한다.
            # 따라서 모든 리스트를 하나의 문자열 리스트로 평탄화(flatten)한 뒤 join한다.
            keyword_str = (
                f"{' '.join(obj)} / "
                f"{' '.join(source)} / "
                f"{' '.join(target)} / "
                f"{' '.join(return_pos)}"
            )

            self.get_logger().warn(f"Detected tools: {keyword_str}")

            with self._lock:
                self._latest_command = keyword_str
            self._command_ready.set()

    def _call_emergency_stop(self):
        """2026-07-07: /voice/estop 토픽에 True를 발행한다 (fire-and-forget).

        실제 로봇 정지와 안전 상태 방송은 safety_monitor_node가 이 토픽을 받아
        수행한다. 발행은 rmw 계층에서 바로 나가므로 여기서 spin을 돌릴 필요가 없다.
        """
        self.estop_pub.publish(Bool(data=True))

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
        """2026-07-06: 여기서 직접 녹음/STT를 하지 않는다.

        _listen_loop가 항상 백그라운드에서 듣고 있다가 만들어둔 최신 명령이
        준비될 때까지(threading.Event) 기다렸다가 그걸 꺼내서 응답으로 돌려준다.
        """
        self._command_ready.wait()

        with self._lock:
            keyword_str = self._latest_command
            self._latest_command = None
        self._command_ready.clear()

        response.success = True
        response.message = keyword_str
        return response


def main():  # d2 메인문 일부 수정
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

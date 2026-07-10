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
from langchain.prompts import PromptTemplate  # d2 이거를 langchain_core로 바꿈
# from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String, Float32
from voice_interface.MicController import MicController, MicConfig

from voice_interface.wakeup_word import WakeupWord
from voice_interface.stt import STT
from voice_interface.tts import TTS

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

# 2026-07-10: HMI(STT-TTS 탭)의 "지금 듣고 있는지/녹음 중인지" 오브 애니메이션용.
# hmi_interface의 voice_bridge 노드가 이 두 토픽을 구독해서 websocket으로 그대로
# 흘려보낸다 - 이 노드는 누가 구독하는지 몰라도 되고(fire-and-forget), Flask가
# ROS를 직접 붙잡지 않는다는 원칙도 그대로 유지된다.
VOICE_STATE_TOPIC = "/voice/state"    # "idle" | "recording" | "processing" | "speaking"
VOICE_LEVEL_TOPIC = "/voice/level"    # Float32, 0.0~1.0 정규화 RMS

# STT 텍스트에 이 단어가 있으면 LLM 왕복을 기다리지 않고 바로 응급정지를 호출한다.
# STOP은 안전(즉시성)이 최우선이라 로컬 문자열 매칭으로 처리한다.
STOP_KEYWORDS = ("정지", "멈춰", "스톱", "중지", "그만")

# 2026-07-09: 유효한 명령을 하나 처리하면 바로 세션이 끝나므로(_listen_loop의 break),
# 이 상한은 STOP이나 파싱 실패(빈 STT 등)만 반복되며 세션이 안 끝나는 경우를 위한
# 안전판으로만 남는다. "빈 STT 결과(침묵)"로 세션을 끝내지 않는 이유는, 노이즈가
# 심한 환경에서는 Whisper가 배경 소음도 뭔가로 전사해버려서 침묵 판정이 잘 안 되기
# 때문 — 그래서 세션 종료 기준을 "침묵"이 아니라 경과 시간으로 건다.
MAX_SESSION_SEC = 5.0

# 2026-07-08: TTS(로봇 음성 응답). brain_node가 이 토픽으로 말할 텍스트를 던지면
# 여기서 재생한다. 웨이크워드 리스너와 같은 노드에 둔 이유는, 로봇이 말하는 동안
# 자기 목소리를 웨이크워드로 오인식하는 피드백 루프를 막으려면 리스너를 잠시
# 멈춰야 하는데(self._speaking) 같은 프로세스에 있어야 그게 간단하기 때문이다.
TTS_TOPIC = "/tts/speak"
TTS_VOICE = "alloy"  # OpenAI TTS 목소리(alloy/echo/fable/onyx/nova/shimmer 등)

# 2026-07-08: RESUME(재개)은 STOP과 반대로 즉시성보다 오탐 방지가 더 중요하다
# (이 마이크 환경은 인식 품질이 낮아 로컬 문자열 매칭만으로 판단하면 잡음을 "다시
# 시작해"로 오인해 의도치 않게 안전정지를 풀어버릴 위험이 있다). 그래서 RESUME은
# STOP처럼 로컬에서 즉시 처리하지 않고, 반드시 LLM(extract_keyword)의 판단을 거쳐
# brain_node로 전달한다 — brain이 실제로 /safety/reset을 호출할지 결정한다.
############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0, openai_api_key=openai_api_key
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

<YOLO 클래스 매핑>
- obj_B → class id 0
- obj_C → class id 1
- hand → class id 2
- obstacle → class id 3
- obj_A → class id 4
- 사용자의 명령에는 class id 번호를 출력하지 않습니다. 항상 obj_A/obj_B/obj_C 이름으로만 출력하세요.

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

<안전 규칙2>
- 사용자가 "다시 시작해", "재개", "계속해", "다시 움직여", "동작해"처럼 정지 상태를
  풀고 다시 동작하라는 취지로 말하면 재개 명령으로 판단합니다.
- 재개 명령이면 객체, 출발지, 목적지, 복귀위치를 모두 RESUME으로 출력하세요.
- 애매하거나 물체 이동 명령과 헷갈리면(예: 목적지가 불명확) RESUME으로 단정하지
  말고 일반 물체 이동 명령 규칙을 우선 적용하세요. RESUME은 정지 해제 의도가
  명확할 때만 출력합니다.

<월드맵 규칙>
- 사용자가 "월드맵 맵핑", "월드맵 매핑", "월드맵 스캔", "월드맵 업데이트"를 말하면 월드맵 작업 명령으로 판단합니다.
- 안전 정지 명령이 포함된 경우에는 월드맵 명령보다 STOP을 우선합니다.
- 월드맵 작업 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 WORLD_MAP으로 출력하세요.

<손 감지 무시 규칙>
- 사용자가 "손 아니야", "손 아님", "그거 손 아니야", "손 감지 무시해", "손 아니라고"처럼
  손 감지 오탐을 정정하는 취지로 말하면 손 감지 무시 명령으로 판단합니다.
- 안전 정지 명령이 포함된 경우에는 이 명령보다 STOP을 우선합니다.
- 손 감지 무시 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 IGNORE_HAND로 출력하세요.

<출력 형식>
- 반드시 아래 형식으로만 출력하세요.
객체 / 출발지 / 목적지 / 복귀위치
STOP / STOP / STOP / STOP
RESUME / RESUME / RESUME / RESUME
IGNORE_HAND / IGNORE_HAND / IGNORE_HAND / IGNORE_HAND

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
- "가져와", "갖다 놔", "이동해", "옮겨", "이동시켜"는 모두 이동 명령으로 간주합니다.
- 출력은 반드시 한 줄만 작성합니다.
- "출력:", "결과:", "설명:" 등의 문구를 절대 포함하지 않습니다.

<객체 매핑>
- "빨간색 통", "빨간거", "빨간색", "빨간 시약" → obj_A
- "파란색 통", "파란거", "파란색", "파란 시약" → obj_B
- "초록색 통", "초록거", "초록색", "초록 시약" → obj_C

<위치 매핑>
- "대기 위치", "대기 지점", "홈", "home" → home
- "처음 위치", "집는 위치", "시작 위치", "시작지점", "스캔 위치", "출발 위치", "0번", "0번 위치" → scan
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
빨간색 통을 이동시켜

출력:
obj_A / scan / UNKNOWN / home

입력:
월드맵 맵핑해

출력:
WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP

입력:
월드맵 스캔 시작

출력:
WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP

입력:
월드맵 업데이트 해줘

출력:
WORLD_MAP / WORLD_MAP / WORLD_MAP / WORLD_MAP

입력:
손 아니야

출력:
IGNORE_HAND / IGNORE_HAND / IGNORE_HAND / IGNORE_HAND

입력:
손 감지 무시해

출력:
IGNORE_HAND / IGNORE_HAND / IGNORE_HAND / IGNORE_HAND

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

입력:
다시 시작해

출력:
RESUME / RESUME / RESUME / RESUME

입력:
재개해줘

출력:
RESUME / RESUME / RESUME / RESUME

입력:
계속해

출력:
RESUME / RESUME / RESUME / RESUME

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

        # 2026-07-10: HMI 오브 애니메이션용 상태/레벨 발행자. VOICE_STATE_TOPIC 주석 참고.
        self.voice_state_pub = self.create_publisher(String, VOICE_STATE_TOPIC, 10)
        self.voice_level_pub = self.create_publisher(Float32, VOICE_LEVEL_TOPIC, 10)
        self._last_published_state = None

        # 2026-07-09: RESUME("다시 시작해")도 STOP처럼 brain_node를 거치지 않고
        # 여기서 바로 처리한다. ESTOP이 이제 액션을 중단시키지 않고 그 자리에서
        # 대기만 하는 구조라(motion_executor._handle_interrupts), brain_node의
        # 메인 루프는 그 액션의 결과를 기다리며 항상 블로킹돼 있다 — 그 상태에서는
        # get_keyword()를 다시 호출할 기회가 없어서, RESUME을 brain_node로 넘기면
        # 영원히 전달이 안 되고 로봇이 안 움직이는 문제가 있었다(실측). LLM 분류는
        # extract_keyword()에서 이미 거치므로(오탐 방지는 그대로 유지), 여기서
        # 결과만 보고 바로 /safety/reset을 호출한다.
        self.safety_reset_client = self.create_client(Trigger, "/safety/reset")

        # 2026-07-09: YOLO hand 감지 오탐(그리퍼+쥔 물체를 손으로 오인식) 임시
        # 완화책. 모델을 다시 학습시키는 대신, "손 아니야"라고 말하면 짧은 시간
        # 동안 hand 감지를 무시하도록 safety_monitor에 요청한다. RESUME과 같은
        # 이유로 brain_node를 거치지 않고 여기서 바로 호출한다.
        self.ignore_hand_client = self.create_client(Trigger, "/safety/ignore_hand")

        # 2026-07-06: 백그라운드 리스닝 스레드(_listen_loop)와 get_keyword() 서비스
        # 콜백이 공유하는 상태. _listen_loop가 파싱한 최신 명령을 여기 넣어두면
        # get_keyword()가 그걸 꺼내서 응답으로 돌려준다.
        self._lock = threading.Lock()
        self._latest_command = None
        self._command_ready = threading.Event()

        # 2026-07-08: TTS. brain_node가 /tts/speak로 던진 텍스트를 재생한다.
        # _speaking은 재생 중임을 알려 웨이크워드 리스너를 잠시 멈추는 플래그.
        #
        # 2026-07-08 (버그 수정): STT(sd.rec)와 TTS(sd.play)는 둘 다 sounddevice의
        # 전역 편의 함수를 쓰는데, 이 함수들은 스레드 세이프하지 않다(내부적으로
        # 전역 스트림 상태를 공유). _listen_loop(백그라운드 스레드)가 STT 녹음
        # 중일 때 ROS spin 스레드에서 _on_speak(TTS)가 동시에 불리면 두 스레드가
        # 동시에 sounddevice 스트림을 건드리게 되어 세그폴트가 났다(실측: get_keyword_node
        # 프로세스 자체가 죽음). 이 락으로 STT 녹음과 TTS 재생을 서로 배타적으로 만든다.
        self._audio_lock = threading.Lock()
        self.tts = TTS(openai_api_key, voice=TTS_VOICE)
        self._speaking = threading.Event()
        self.create_subscription(String, TTS_TOPIC, self._on_speak, 10)

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
        일반 명령(RESUME 포함)은 self._latest_command에 저장해 get_keyword()가
        꺼내가게 한다 — RESUME은 STOP과 달리 여기서 즉시 처리하지 않고 반드시
        LLM 분류 결과를 brain_node로 넘겨서 처리한다(오탐 방지).

        2026-07-09: 웨이크워드 한 번당 유효한 명령 하나만 처리하고 바로 세션을
        끝낸다(다음 명령은 다시 웨이크워드 필요). 예전엔 MAX_SESSION_SEC 동안
        연달아 여러 명령을 받았는데, 그러면 brain_node가 이전 명령을 아직 소비
        (get_keyword 서비스로 꺼내가기)하기도 전에 다음 발화가 self._latest_command를
        덮어써서 명령이 씹히는 문제가 있었다(_latest_command가 큐가 아니라 슬롯
        하나뿐이라서). STOP은 예외 — 안전이 우선이라 세션 안에서 계속 들을 수 있게
        둔다(이 부분은 안 바꿈). MAX_SESSION_SEC는 STOP/파싱실패만 반복될 때의
        세션 상한으로만 남는다.
        """
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return

        self._publish_voice_state("idle")

        while rclpy.ok():
            # 2026-07-08: TTS 재생 중에는 웨이크워드 판정을 멈춘다 — 안 그러면
            # 로봇이 말하는 자기 목소리를 웨이크워드로 오인식하는 피드백 루프가
            # 생긴다. 재생이 끝나면 _on_speak가 이 플래그를 풀고 마이크 버퍼도
            # 비워준다.
            if self._speaking.is_set():
                time.sleep(0.05)
                continue

            woke = self.wakeup_word.is_wakeup()
            # 2026-07-10: 웨이크워드 대기 중에도 HMI 오브가 주변 소리에 반응하도록
            # is_wakeup()이 매 청크마다 갱신해둔 레벨을 그대로 흘려보낸다.
            self._publish_voice_level(self.wakeup_word.last_level)
            if not woke:
                continue

            self.get_logger().info("헬로우 로키 감지 - 녹음 시작")

            # 웨이크워드 세션: MAX_SESSION_SEC 동안은 재웨이크 없이 계속 듣는다.
            session_start = time.time()
            while rclpy.ok() and (time.time() - session_start) < MAX_SESSION_SEC:
                # STT --> Keyword Extract --> Embedding
                self._publish_voice_state("recording")
                with self._audio_lock:
                    output_message = self.stt.speech2text(level_callback=self._publish_voice_level)
                self._flush_mic_stream()
                self.get_logger().info(f"STT 인식 결과: '{output_message}'")

                # Whisper는 무음/잡음 구간에서 빈 문자열이나 아주 짧은 환청을 자주
                # 낸다. 이번 라운드만 건너뛰고(세션은 유지, 시간으로만 종료) LLM에
                # 안 넣는다 — 그대로 넣으면 UNKNOWN이나 엉뚱한 STOP으로 파싱돼
                # 오작동할 수 있다.
                if len(output_message.strip()) < 2:
                    self.get_logger().warn("STT 결과가 비었/너무 짧음 — 이번 라운드 무시")
                    continue

                if any(word in output_message for word in STOP_KEYWORDS):
                    self.get_logger().warn(f"정지 키워드 감지(로컬): '{output_message}' -> 응급정지 호출")
                    self._call_emergency_stop()
                    continue

                self._publish_voice_state("processing")
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

                if "RESUME" in obj:
                    # 2026-07-09: brain_node로 안 넘기고 여기서 바로 처리 —
                    # _call_safety_reset() 주석 참고(brain_node 메인 루프 블로킹 문제).
                    self.get_logger().warn(f"재개 명령 감지(LLM): STT='{output_message}'")
                    self._call_safety_reset()
                    self._speak_locally("다시 시작합니다")
                    break

                if "IGNORE_HAND" in obj:
                    # 2026-07-09: hand 감지 오탐(YOLO) 임시 완화 — RESUME과 같은
                    # 이유로 brain_node를 거치지 않고 여기서 바로 처리한다.
                    self.get_logger().warn(f"손 감지 무시 명령(LLM): STT='{output_message}'")
                    self._call_ignore_hand()
                    self._speak_locally("손 아닌 걸로 하겠습니다")
                    break

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
                break  # 유효한 명령 하나 처리했으니 세션 종료 — 다음 명령은 재웨이크 필요

            self._publish_voice_state("idle")

            # 2026-07-08: 세션 루프 안에서 speech2text() 직후에 한 번 플러시해도,
            # 그 뒤 LLM 호출(extract_keyword, 네트워크 왕복이라 몇 초 걸림) 동안
            # 새로 쌓인 오디오가 안 비워진 채로 세션이 끝나버린다. 그래서 웨이크워드
            # 대기로 돌아가기 직전에 한 번 더 비운다(실측: 이게 없으면 명령 처리
            # 끝나자마자 안 부른 웨이크워드가 또 감지됨).
            self._flush_mic_stream()
            # 2026-07-09: 위 플러시는 원본 오디오 버퍼만 비운다 — openWakeWord
            # 모델 내부의 예측/피처 히스토리 버퍼는 안 비워져서, STT+LLM 처리
            # 몇 초 동안 predict()가 안 불리다가 세션 끝나고 다시 불리면 그
            # 불연속으로 여전히 오탐(실측: 처리 완료 67ms 만에 재감지)이 났다.
            # 모델 자체를 리셋해서 항상 "방금 시작한" 상태로 판단하게 한다.
            self.wakeup_word.reset()

    def _flush_mic_stream(self):
        """STT(sounddevice) 녹음 동안 밀린 PyAudio 웨이크워드 스트림 버퍼를 비운다.

        2026-07-08: sd.rec()가 5초+ 동안 마이크를 붙잡는 사이 self.mic_controller.stream
        (PyAudio, 웨이크워드용)은 아무도 read()하지 않아 내부 버퍼가 밀리거나
        overflow(exception_on_overflow=False라 조용히 드롭)된다. 그 상태로 바로
        is_wakeup() 루프를 재개하면 밀리거나 끊긴 오디오가 모델에 들어가 웨이크워드를
        말하지 않았는데도 감지되는 오탐이 생겼다(재현: 명령 처리 30ms 후 오탐).
        그래서 STT가 끝날 때마다 그 사이 쌓인 걸 다 읽어서 버리고 새로 시작한다.
        """
        try:
            stream = self.mic_controller.stream
            available = stream.get_read_available()
            if available > 0:
                stream.read(available, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().warn(f"마이크 버퍼 플러시 실패(무시): {e}")

    def _publish_voice_state(self, state):
        """HMI 오브 상태 발행. 값이 바뀔 때만 로그를 남기되(스팸 방지), 발행 자체는
        상태 전이 시점마다 한다 - 구독자(hmi_interface.voice_bridge)가 늦게 붙어도
        다음 전이 때 바로 최신값을 받도록 QoS는 기본(volatile)으로 둔다. HMI가
        연결 직후 "idle"을 못 받는 정도는 치명적이지 않다(다음 전이 때 곧 옴)."""
        if state != self._last_published_state:
            self.get_logger().debug(f"[voice] state -> {state}")
            self._last_published_state = state
        self.voice_state_pub.publish(String(data=state))

    def _publish_voice_level(self, level):
        self.voice_level_pub.publish(Float32(data=float(level)))

    def _call_emergency_stop(self):
        """2026-07-07: /voice/estop 토픽에 True를 발행한다 (fire-and-forget).

        실제 로봇 정지와 안전 상태 방송은 safety_monitor_node가 이 토픽을 받아
        수행한다. 발행은 rmw 계층에서 바로 나가므로 여기서 spin을 돌릴 필요가 없다.
        """
        self.estop_pub.publish(Bool(data=True))

    def _speak_locally(self, text):
        """텍스트를 음성으로 재생한다. _on_speak(브레인이 던진 텍스트)와
        _call_safety_reset(RESUME 로컬 처리) 양쪽이 공유하는 헬퍼다.

        재생 동안 self._speaking을 세워 웨이크워드 리스너를 멈추고(자기 목소리
        오탐 방지), 끝나면 그 사이 마이크에 쌓인 오디오(로봇 목소리 포함)를
        버린 뒤 리스너를 재개한다.
        """
        text = text.strip()
        if not text:
            return
        self._speaking.set()
        self._publish_voice_state("speaking")
        try:
            with self._audio_lock:
                self.tts.speak(text)
        except Exception as e:
            self.get_logger().error(f"TTS 재생 실패(무시): {e}")
        finally:
            self._flush_mic_stream()
            self._speaking.clear()
            self._publish_voice_state("idle")

    def _on_speak(self, msg):
        """brain_node가 /tts/speak로 던진 텍스트를 음성으로 재생한다.

        이 콜백은 spin 스레드에서 돌며 재생 시간만큼 블로킹하지만, 그동안
        처리 못 하는 다른 콜백이 없어(get_keyword는 브레인이 말하는 시점엔
        호출 중이 아님) 문제없다.
        """
        self._speak_locally(msg.data)

    def _call_safety_reset(self):
        """RESUME("다시 시작해")을 brain_node를 거치지 않고 여기서 바로 처리한다.

        2026-07-09: safety_reset_client 옆 주석 참고 — ESTOP이 액션을 중단시키지
        않고 그 자리에서 대기만 하는 구조가 되면서, brain_node의 메인 루프가 그
        액션 결과를 기다리며 항상 블로킹돼 있어 RESUME을 brain_node로 넘기면
        전달이 안 됐다(실측: "다시 시작해"를 해도 하강이 재개되지 않음). 여기서
        직접 /safety/reset을 호출해 ESTOP 래치를 해제하면, motion_executor의
        _handle_interrupts가 즉시 감지하고 하던 동작을 재개한다.
        """
        if not self.safety_reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/safety/reset 서비스 없음 — RESUME 처리 실패")
            return
        future = self.safety_reset_client.call_async(Trigger.Request())
        start = time.time()
        while not future.done() and time.time() - start < 2.0:
            time.sleep(0.01)

    def _call_ignore_hand(self):
        """"손 아니야"를 brain_node를 거치지 않고 여기서 바로 처리한다.

        hand 감지는 래치가 아니라 /hand_detected를 실시간으로 반영하므로
        (safety_monitor._current_state), RESUME처럼 한 번 리셋하는 방식이 아니라
        일정 시간 동안 무시하도록 safety_monitor에 요청한다.
        """
        if not self.ignore_hand_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/safety/ignore_hand 서비스 없음 — 처리 실패")
            return
        future = self.ignore_hand_client.call_async(Trigger.Request())
        start = time.time()
        while not future.done() and time.time() - start < 2.0:
            time.sleep(0.01)

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

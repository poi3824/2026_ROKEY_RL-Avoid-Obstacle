# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
import time
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate  # d2 이거를 langchain_core로 바꿈
# from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############

#----------------------------------------------------------------
# current_dir = os.getcwd()
# package_path = get_package_share_directory("pick_and_place_voice")

# env_path = "/home/rokey/cobot_ws/src/cobot2_ws/pick_and_place_voice/resource/.env"
# load_dotenv(dotenv_path=env_path)
# is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
# openai_api_key = os.getenv("OPENAI_API_KEY")
#-----------------------------------------------------------------

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

if openai_api_key is None:
    raise RuntimeError(f"OPENAI_API_KEY not found: {ENV_PATH}")

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
- 객체와 위치는 아래 리스트의 이름만 사용합니다.
- 단, 목적지가 명시되지 않은 경우 목적지는 UNKNOWN을 사용합니다.

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

주의:
- 사용자의 명령에는 class id 번호를 출력하지 않습니다.
- 출력에는 반드시 obj_A, obj_B, obj_C, WORLD_MAP, STOP 중 필요한 값만 사용합니다.
- 색상과 객체 이름의 관계는 아래 객체 매핑을 최우선으로 따릅니다.

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

<안전 규칙>
- 사용자가 "멈춰", "정지", "스톱", "중지", "그만"이라고 말하면 즉시 안전 정지 명령으로 판단합니다.
- 안전 정지 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 STOP으로 출력하세요.
- 입력에 안전 정지 단어가 포함되면 다른 모든 명령보다 STOP을 최우선으로 출력합니다.

<월드맵 규칙>
- 사용자가 "월드맵 맵핑", "월드맵 매핑", "월드맵 스캔", "월드맵 업데이트"를 말하면 월드맵 작업 명령으로 판단합니다.
- 안전 정지 명령이 포함된 경우에는 월드맵 명령보다 STOP을 우선합니다.
- 월드맵 작업 명령이면 객체, 출발지, 목적지, 복귀 위치를 모두 WORLD_MAP으로 출력하세요.

<출력 형식>
객체 / 출발지 / 목적지 / 복귀위치

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
- 지정된 출력 형식 외에는 어떠한 문자도 출력하지 않습니다.

<객체 매핑>
- "빨간색 통", "빨간거", "빨간색", "빨간 시약" → obj_A
- "파란색 통", "파란거", "파란색", "파란 시약" → obj_B
- "초록색 통", "초록거", "초록색", "초록 시약" → obj_C

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
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

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

    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함    # d2 이 함수 일부 수정함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            response.success = False
            response.message = "Failed to open audio stream"
            return response

        while not self.wakeup_word.is_wakeup():
            time.sleep(0.01)

        # STT --> Keyword Extract --> Embedding
        output_message = self.stt.speech2text()
        obj, source, target, return_pos = self.extract_keyword(output_message)

        # extract_keyword가 파싱에 실패해 None을 반환한 경우 방어 처리
        # (원래 코드는 이 경우에도 " ".join(keyword)를 호출해 TypeError가 발생했음)
        if obj is None:
            self.get_logger().error("Failed to extract keyword from LLM response")
            response.success = False
            response.message = "Failed to extract keyword"
            return response

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

        # 응답 객체 설정
        response.success = True
        response.message = keyword_str  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():  # d2 메인문 일부 수정
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# hmi/schemas/*.json을 로드하고 검증하는 얇은 헬퍼.
#
# Python과 TypeScript(React)가 같은 .py/.ts 파일을 직접 import할 수 없기 때문에
# JSON Schema를 공통 계약으로 쓴다(hmi/schemas/ 참고) - 여기서는 그중 Python(백엔드)
# 쪽 런타임 검증만 담당한다. task_status_event.schema.json이 같은 디렉토리의
# task_status.schema.json을 $ref로 참조하므로 RefResolver로 상대 참조를 풀어준다.
import glob
import json
import os
from functools import lru_cache

import jsonschema

SCHEMAS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "schemas"
)


@lru_cache(maxsize=None)
def _load(schema_filename):
    path = os.path.join(SCHEMAS_DIR, schema_filename)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


@lru_cache(maxsize=None)
def _resolver():
    """모든 스키마를 $id 기준으로 미리 읽어 store에 채워두는 공유 리졸버.
    task_status_event.schema.json이 $ref: "task_status.schema.json"으로 같은
    디렉토리의 스키마를 참조하는데, 각 스키마가 자체 $id(https://hmi.local/...)를
    갖고 있어서 RefResolver가 그 $id를 base URI로 삼아 실제 네트워크에서 가져오려는
    문제가 있었다(오프라인 환경에서 DNS 에러) - store를 미리 채워 네트워크 접근
    자체를 막는다."""
    store = {}
    for path in glob.glob(os.path.join(SCHEMAS_DIR, "*.schema.json")):
        with open(path, "r", encoding="utf-8") as f:
            doc = json.load(f)
        schema_id = doc.get("$id")
        if schema_id:
            store[schema_id] = doc
    return jsonschema.RefResolver(base_uri="", referrer={}, store=store)


def validate(schema_filename, payload):
    """payload가 스키마를 어기면 jsonschema.ValidationError를 던진다."""
    schema = _load(schema_filename)
    jsonschema.validate(instance=payload, schema=schema, resolver=_resolver())

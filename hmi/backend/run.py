# 개발용 실행 진입점: python run.py (hmi/backend/ 안에서 실행).
#
# 프로덕션에서는 socketio.run()의 내장 Werkzeug dev 서버를 쓰지 않는다 - gunicorn +
# eventlet(또는 별도 WSGI 서버) 뒤에서 돌리고, React는 `npm run build` 결과물을
# Flask가 정적 파일로 서빙(또는 별도 웹서버에서 서빙 후 API/Socket.IO만 이 프로세스가
# 담당)하는 구조로 바꾼다. Phase 0에서는 dev 서버로 충분하다.
from app import create_app
from config import Config

if __name__ == "__main__":
    app, socketio = create_app()
    socketio.run(
        app,
        host=Config.HOST,
        port=Config.PORT,
        debug=Config.DEBUG,
        allow_unsafe_werkzeug=True,
    )

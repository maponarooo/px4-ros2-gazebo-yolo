import asyncio
import websockets
import json
import base64
import cv2
import numpy as np

# WebSocket 서버 주소
SERVER_URI = "ws://172.21.77.220:8765"  # <=== 실제 서버 IP로 바꾸세요

async def receive_frames():
    async with websockets.connect(SERVER_URI) as websocket:
        print("✅ WebSocket 서버에 연결됨")

        while True:
            try:
                # 서버로부터 메시지 수신
                message = await websocket.recv()
                data = json.loads(message)

                # 이미지 디코딩
                frame_b64 = data.get("frame", "")
                frame_bytes = base64.b64decode(frame_b64)
                nparr = np.frombuffer(frame_bytes, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                # 객체 인식 정보 출력
                objects = data.get("objects", [])
                for obj in objects:
                    label = obj["label"]
                    confidence = obj["confidence"]
                    x, y, w, h = obj["bbox"]
                    print(f"[{label}] ({confidence}) at ({x},{y},{w},{h})")

                    # 바운딩 박스 시각화
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(img, f"{label} {confidence}",
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)

                # 프레임 표시
                cv2.imshow("YOLO Stream", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print("❌ 오류:", e)
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_frames())

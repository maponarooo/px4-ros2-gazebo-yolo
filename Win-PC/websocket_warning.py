import asyncio
import websockets
import json
import base64
import cv2
import numpy as np
import time
import platform
import threading

if platform.system() == "Windows":
    import winsound

SERVER_URI = "ws://172.21.77.220:8765"
ALERT_DURATION = 1.0

# 스레드 중단을 위한 이벤트 플래그
stop_alert_event = threading.Event()
alert_thread = None

def alert_sound_loop():
    while not stop_alert_event.is_set():
        if platform.system() == "Windows":
            winsound.Beep(1000, 300)  # 300ms 경고음
        else:
            print("\a")  # Linux/macOS 터미널 beep
        time.sleep(0.2)  # 반복 간격

def start_alert_sound():
    global alert_thread
    if alert_thread is None or not alert_thread.is_alive():
        stop_alert_event.clear()
        alert_thread = threading.Thread(target=alert_sound_loop, daemon=True)
        alert_thread.start()

def stop_alert_sound():
    stop_alert_event.set()

async def receive_frames():
    async with websockets.connect(SERVER_URI) as websocket:
        print("✅ WebSocket 서버에 연결됨")

        alert_active = False
        alert_start_time = 0

        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)

                # 이미지 디코딩
                frame_b64 = data.get("frame", "")
                frame_bytes = base64.b64decode(frame_b64)
                nparr = np.frombuffer(frame_bytes, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                # 객체 인식
                objects = data.get("objects", [])
                intruder_detected = len(objects) > 0

                for obj in objects:
                    label = obj["label"]
                    confidence = obj["confidence"]
                    x, y, w, h = obj["bbox"]
                    print(f"[{label}] ({confidence}) at ({x},{y},{w},{h})")
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(img, f"{label} {confidence:.2f}",
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)

                # 경고 상태 진입 또는 유지
                if intruder_detected:
                    if not alert_active:
                        print("🚨 침입자 감지! 경고음 재생 시작")
                        start_alert_sound()
                    alert_active = True
                    alert_start_time = time.time()
                else:
                    if alert_active:
                        print("✅ 감지 종료, 경고음 중단")
                        stop_alert_sound()
                    alert_active = False

                # 시각적 경고
                if alert_active:
                    overlay = img.copy()
                    red_overlay = np.full(img.shape, (0, 0, 255), dtype=np.uint8)
                    alpha = 0.4
                    cv2.addWeighted(red_overlay, alpha, overlay, 1 - alpha, 0, overlay)
                    cv2.putText(overlay, "🚨 Intruder Warning!!! 🚨",
                                (50, 100), cv2.FONT_HERSHEY_DUPLEX, 1.5,
                                (255, 255, 255), 3, cv2.LINE_AA)
                    img = overlay

                # 영상 출력
                cv2.imshow("YOLO Stream", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print("❌ 오류:", e)
                break

    stop_alert_sound()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_frames())

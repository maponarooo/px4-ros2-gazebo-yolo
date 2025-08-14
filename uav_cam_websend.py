# ROS2 + YOLO + WebSocket 통합 버전
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import asyncio
import websockets
import base64
import json
import threading

# YOLO 모델 로드
model = YOLO('yolo11n.pt')

# 전역: 최신 프레임 저장용 변수
latest_frame_data = {
    "image": None,
    "objects": []
}

# WebSocket 서버 루프
async def websocket_handler(websocket):
    while True:
        try:
            if latest_frame_data["image"] is not None:
                # JSON 직렬화 안전화
                safe_objects = []
                for obj in latest_frame_data["objects"]:
                    safe_obj = {
                        "label": obj["label"],
                        "confidence": float(obj["confidence"]),
                        "bbox": list(map(int, obj["bbox"]))
                    }
                    safe_objects.append(safe_obj)

                data = {
                    "frame": latest_frame_data["image"],
                    "objects": safe_objects
                }

                await websocket.send(json.dumps(data))

            await asyncio.sleep(0.1)
        except Exception as e:
            print("❌ WebSocket 전송 중 오류:", e)
            break

# WebSocket 서버 실행 함수
def start_websocket_server():
    import asyncio
    import websockets

    async def run_websocket_server():
        async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
            print("✅ WebSocket 서버가 8765 포트에서 실행 중입니다.")
            await asyncio.Future()  # 서버가 영원히 실행되도록 블로킹

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_websocket_server())

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.subscription
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('📷 프레임 수신 중...')

        # ROS 이미지 → OpenCV
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # YOLO 객체 인식
        results = model.predict(current_frame, classes=[0, 2], imgsz=640, conf=0.3)
        img = results[0].plot()

        # bounding box 추출
        objects = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                objects.append({
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": [x1, y1, x2 - x1, y2 - y1]
                })

        # 이미지 인코딩
        _, jpeg = cv2.imencode('.jpg', img)
        frame_b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')

        # 전역 변수에 저장 (WebSocket에서 접근)
        latest_frame_data["image"] = frame_b64
        latest_frame_data["objects"] = objects

        # 로컬 표시
        cv2.namedWindow('Detected Frame', flags=cv2.WINDOW_NORMAL)
        cv2.imshow('Detected Frame', img)
        cv2.waitKey(1)


def main(args=None):
    # WebSocket 서버 백그라운드 실행
    t = threading.Thread(target=start_websocket_server, daemon=True)
    t.start()

    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

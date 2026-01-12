# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class MultiCompressedWebcamPublisher(Node):
#     def __init__(self):
#         super().__init__('multi_compressed_webcam_publisher')
        
#         # 1. 사용할 카메라 인덱스 리스트 (0, 1, 2, 3번 카메라)
#         # USB 대역폭 문제로 4개가 동시에 안 열릴 경우 해상도를 낮추거나 USB 포트를 분산해야 합니다.
#         self.camera_indices = [0, 1, 2]
#         self.cameras = []

#         # JPEG 압축 퀄리티 설정
#         self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        
#         # 2. 각 카메라별 캡처 객체와 퍼블리셔 생성
#         for idx in self.camera_indices:
#             cap = cv2.VideoCapture(idx)
            
#             # 카메라가 정상적으로 열렸는지 확인
#             if cap.isOpened():
#                 # 해상도 설정 (4개를 동시에 돌리려면 대역폭 관리를 위해 낮추는 것이 좋습니다)
#                 cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#                 cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                
#                 # 토픽 이름을 구분 (예: /camera_0/image/compressed)
#                 topic_name = f'/camera_{idx}/image/compressed'
#                 publisher = self.create_publisher(CompressedImage, topic_name, 10)
                
#                 self.cameras.append({
#                     'id': idx,
#                     'cap': cap,
#                     'pub': publisher
#                 })
#                 self.get_logger().info(f'Camera {idx} initialized on topic {topic_name}')
#             else:
#                 self.get_logger().error(f'Could not open camera {idx}')

#         timer_period = 0.1  # 10Hz
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.bridge = CvBridge()

#     def timer_callback(self):
#         # 등록된 모든 카메라를 순회하며 이미지 발행
#         for cam in self.cameras:
#             ret, frame = cam['cap'].read()
            
#             if ret:
#                 # OpenCV 이미지를 JPEG로 압축
#                 success, encoded_image = cv2.imencode('.jpg', frame, self.encode_param)
                
#                 if success:
#                     msg = CompressedImage()
#                     msg.header.stamp = self.get_clock().now().to_msg()
#                     msg.header.frame_id = f"camera_frame_{cam['id']}" # Frame ID도 구분
#                     msg.format = "jpeg"
#                     msg.data = np.array(encoded_image).tobytes()
                    
#                     # 해당 카메라의 퍼블리셔로 발행
#                     cam['pub'].publish(msg)
#             else:
#                 self.get_logger().warning(f"Camera {cam['id']}: 프레임을 읽을 수 없습니다.")

#     def __del__(self):
#         # 모든 카메라 자원 해제
#         for cam in self.cameras:
#             if cam['cap'].isOpened():
#                 cam['cap'].release()

# def main(args=None):
#     rclpy.init(args=args)
#     node = MultiCompressedWebcamPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

##########################################3333
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedWebcamPublisher(Node):
    def __init__(self):
        super().__init__('compressed_webcam_publisher')
        
        # 1. CompressedImage 메시지 타입 사용
        self.publisher_ = self.create_publisher(CompressedImage, '/image/compressed', 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        # JPEG 압축 퀄리티 설정 (0~100, 높을수록 고화질/대용량)
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # 2. OpenCV 이미지를 JPEG로 압축 (Encoding)
            success, encoded_image = cv2.imencode('.jpg', frame, self.encode_param)
            
            if success:
                # 3. CompressedImage 메시지 생성 및 채우기
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = np.array(encoded_image).tobytes()
                
                self.publisher_.publish(msg)
        else:
            self.get_logger().warning("프레임을 읽을 수 없습니다.")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CompressedWebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''

# Subscriber 콜백 함수 예시
def listener_callback(self, msg):
    # 1. 바이트 데이터를 numpy 배열로 변환
    np_arr = np.frombuffer(msg.data, np.uint8)
    
    # 2. 이미지 디코딩 (압축 해제)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    cv2.imshow("Received", image_np)
    cv2.waitKey(1)
'''
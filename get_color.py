import rclpy ,os, time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


class ColorBasedParameterUpdater(Node):
    def __init__(self):
        super().__init__('color_based_param_updater')
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.get_img_cb, 10)
        self.cv_img = None
        self.bridge = CvBridge()

    def get_img_cb(self, msg):
        self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    def process_frame(self, frame):
    # HSV로 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 넓은 범위로 색상 정의 (모든 주요 RGB를 포함)
        color_ranges = {
            'blue': (np.array([90, 50, 50]), np.array([130, 255, 255])),  # 파란색
            'red': [
                (np.array([0, 50, 50]), np.array([10, 255, 255])),        # 빨간색 (첫 번째 범위)
                (np.array([170, 50, 50]), np.array([180, 255, 255]))     # 빨간색 (두 번째 범위)
            ],
            'green': (np.array([40, 50, 50]), np.array([80, 255, 255]))   # 초록색
        }

        detected_color = None
        largest_area = 0

        # 색상별로 탐지 및 영역 계산
        for color, ranges in color_ranges.items():
            if isinstance(ranges, list):  # 빨간색처럼 두 범위를 병합해야 하는 경우
                mask = cv2.inRange(hsv, ranges[0][0], ranges[0][1]) | cv2.inRange(hsv, ranges[1][0], ranges[1][1])
            else:
                mask = cv2.inRange(hsv, ranges[0], ranges[1])

            res = cv2.bitwise_and(frame, frame, mask=mask)

            # 이진화 후 외곽선 찾기
            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            _, bin = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > largest_area:  # 가장 큰 영역의 색상 선택
                    largest_area = area
                    detected_color = color

        # 결과 업데이트
        self.update_parameters(detected_color)


    def update_parameters(self, detected_color):
        if detected_color is None:
            self.get_logger().info('No color detected.')
        else:
            self.get_logger().info(f'Detected color: {detected_color}')
            os.system(f"ros2 param set /reg_params color {detected_color}")
            os.system("ros2 param set /reg_params go_stop go")
            # Exit the program after updating parameters
            self.get_logger().info("Parameters updated. Shutting down node.")
            rclpy.shutdown()

    def run(self):
        try:
            while self.cv_img is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)  # Prevent CPU overloading

            self.get_logger().info("Camera input received. Processing frames...")


            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.cv_img is not None and self.cv_img.size > 0:
                    frame = self.cv_img.copy()
                    self.process_frame(frame)
                    cv2.imshow("VideoFrame", self.cv_img)
                else:
                    self.get_logger().warn("No valid image received from the camera.")

        except KeyboardInterrupt:
            self.get_logger().info('Keyboard Interrupt (SIGINT)')

        finally:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ColorBasedParameterUpdater()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

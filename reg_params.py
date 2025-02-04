import rclpy, sys, os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import sys, select

class RegParams(Node):
    def __init__(self):
        super().__init__('reg_params')

        # 기본 파라미터 선언
        self.declare_parameter('arrive_1', 'yet')
        self.declare_parameter('color', None)
        self.declare_parameter('go_stop', 'stop')
        self.declare_parameter('back', 'stop')

        self.get_logger().info("키보드 입력 대기 중... (0을 입력하면 파라미터 초기화)")

    def reset_parameters(self):
        """ 파라미터 값을 기본값으로 초기화 """
        os.system("ros2 param set /reg_params arrive_1 yet")
        os.system("ros2 param set /reg_params color None")
        os.system("ros2 param set /reg_params go_stop stop")
        os.system("ros2 param set /reg_params back stop")
        self.get_logger().info("🔄 모든 파라미터가 초기화되었습니다.")

    def wait_for_input(self):
        """ 키보드 입력을 감지하여 0을 입력하면 초기화 수행 """
        while rclpy.ok():
            print("입력 대기 중... (0을 입력하면 초기화)")
            select.select([sys.stdin], [], [], 0)  # 입력 감지
            user_input = sys.stdin.read(1).strip()  # 한 글자 입력받기
            
            if user_input == '0':
                self.reset_parameters()  # 파라미터 초기화
            else:
                print(f"입력값: {user_input} (0을 입력해야 초기화됩니다.)")

def main():
    rclpy.init()
    node = RegParams()

    try:
        node.wait_for_input()
    except KeyboardInterrupt:
        print("\n프로그램 종료.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy, sys, os, time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
from project1.move_tb3 import MoveTB3

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075


class ClientFollowPoints(Node):

    def __init__(self):
        super().__init__('client_follow_points')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('go_stop', 'stop')
        self.declare_parameter('color', None)

        self.pub_lift = self.create_publisher(String, '/lift_msg', 10)
        self.points = None
        self.is_goal_done = False  # 결과 처리가 완료되었는지 상태 플래그
        self.tb3 = MoveTB3()
        self.tw = Twist()

    def send_points(self, points):
        """Action 서버에 waypoints 전송"""
        msg = FollowWaypoints.Goal()
        msg.poses = points

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """목표 지점 도달 여부 확인 및 후속 명령 실행"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("목표 지점에 도착했습니다!")

            # 목표 지점 도착 후 후속 명령 실행
            self.pub_lift_msg("lift_down")

            os.system('ros2 param set /back_move back go')   

            time.sleep(2)  # 메시지 발행 후 대기

            
            
        else:
            self.get_logger().warn("목표 지점 도달에 실패했습니다!")


            self.get_logger().info('결과: {0}'.format(result.missed_waypoints))

        self.is_goal_done = True  # 작업 완료 상태 플래그 설정
            

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'피드백 수신: {feedback.current_waypoint}')

    def pub_lift_msg(self, lift_msg):
        """리프트 명령 메시지 발행"""
        msg = String()
        msg.data = lift_msg
        self.pub_lift.publish(msg)

    def set_waypoints_based_on_color(self, color):
        """감지된 색에 따라 이동 좌표 설정"""
        if color == "red":
            x, y, w = 1.2, 0.6, 1.0
        elif color == "green":
            x, y, w = 1.0, 0.0, 1.0
        elif color == "blue":
            x, y, w = 1.0, -0.4, 1.0
        else:
            self.get_logger().warn(f"알 수 없는 색상: {color}")
            self.points = None
            return

        rgoal = PoseStamped()
        rgoal.header.frame_id = "map"
        rgoal.header.stamp.sec = 0
        rgoal.header.stamp.nanosec = 0
        rgoal.pose.position.x = x
        rgoal.pose.position.y = y
        rgoal.pose.position.z = w
        rgoal.pose.orientation.w = 1.0

        self.points = [rgoal]
        self.get_logger().info(f"색상 {color}에 대한 좌표 설정: {x}, {y}")

    def main_loop(self):
        """색상 및 go_stop 파라미터를 모니터링하고 동작 실행"""
        while rclpy.ok():
            color = self.get_parameter('color').value
            go_stop = self.get_parameter('go_stop').value

            if color:
                self.set_waypoints_based_on_color(color)

            if go_stop == 'go' and self.points:
                self.get_logger().info("go_stop이 'go'로 설정됨. 이동 시작!")
                self.send_points(self.points)

                # 결과 처리가 완료될 때까지 대기
                while not self.is_goal_done:
                    self.get_logger().info("목표 지점 도달 상태 확인 중...")
                    rclpy.spin_once(self, timeout_sec=1.0)
                break  # 후속 작업 완료 후 루프 종료

            else:
                self.get_logger().info("색상과 'go_stop' 확인 중...")
                rclpy.spin_once(self, timeout_sec=1.0)

         
        

def main(args=None):
    rclpy.init(args=args)

    node = ClientFollowPoints()
    print('클라이언트 초기화 완료')

    node.main_loop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

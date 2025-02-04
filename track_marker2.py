import rclpy, sys, os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion #, quaternion_from_euler
from project1.move_tb3 import MoveTB3

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

R = 1.5708


class TrackMarker(Node):
    """   
                                                    ////////////| ar_marker |////////////
            y                      z                --------+---------+---------+--------
            ^  x                   ^                        |     R-0/|\R-0    R|
            | /                    |                        |       /0|0\       |
     marker |/                     | robot                  |      /  |  \      |
            +------> z    x <------+                        |     /   |   \     |
                                  /                         |  dist   |  dist   |
                                 /                          |   /     |     \   |
                                y                           |  /      |      \  |
                                                            | /       |       \0|
                                                            |/R-0    R|R    R-0\|
    pose.x = position.z                             (0 < O) x---------+---------x (0 > O)
    pose.y = position.x              [0]roll    (pos.x > O) ^                   ^ (pos.x < O)
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose  = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
        
        self.declare_parameter('arrive_1', 'off')
            
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_lift = self.create_publisher(String, '/lift_msg', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)
        
        self.pose = Pose()
        self.tw   = Twist()
        self.tb3  = MoveTB3()
        self.lift_msg = String()
        
        self.theta   = 0.0
        self.dir     = 0
        self.th_ref  = 0.0
        self.z_ref   = 0.0
        self.cnt_sec = 0
        
        self.target_found = False        
        
    def get_marker_pose_(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        if len(msg.marker_ids) == 0:  # No marker found
            self.target_found = False
            self.get_logger().info("No markers found.")
        
        else:  # At least one marker found
            self.target_found = True
            self.pose = msg.poses[0]  # 첫 번째 마커의 pose 사용
            self.theta = self.get_theta(self.pose)
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta
    
    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1    
        
    def pub_lift_msg(self, lift_msg):
        msg = String()
        msg.data = lift_msg
        self.pub_lift.publish(msg)
    
    def stop_move(self):
        self.tw.linear.z = self.tw.angular.z = 0.0
        self.pub_tw.publish(self.tw)      

    def reset_marker_info(self):
        self.target_found = False
        self.pose = Pose()
        self.theta = 0.0
        self.get_logger().info("Marker information has been reset.")

    def main_loop(self):
        # 메인 루프에서 파라미터 상태 확인
        while rclpy.ok():
            arrive_1 = self.get_parameter('arrive_1').value  # 파라미터 읽기
            if arrive_1 == 'arrived':
                self.get_logger().info("arrive_1 is 'arrived'.")
                self.reset_marker_info() #동작 실행 전 마커정보 초기화 
                # 동작 실행
                self.execute_movement()
            else:
                self.get_logger().info("Waiting for 'arrive_1' to be 'arrived'.")
                rclpy.spin_once(self, timeout_sec=1.0)
        
    def execute_movement(self):
        self.tw.angular.z = ANG_SPEED
    
        while rclpy.ok():
            if self.theta != 0.0:   break   # this means target marker found
            self.pub_tw.publish(self.tw)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_move()
        print("\n----- 1_target marker found!\n") ###########################
        
        while self.pose.position.x < -0.0175 or self.pose.position.x >  0.0175:
            rclpy.spin_once(self, timeout_sec=0.1)
            if   self.pose.position.x < -0.0155:
                self.tw.angular.z =  0.175 * ANG_SPEED
            else:# self.pose.position.x >  0.025:
                self.tw.angular.z = -0.125 * ANG_SPEED
            self.pub_tw.publish(self.tw)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_move()        
        print("\n----- 2_arrived reference position!\n") ####################
        
        self.th_ref = self.theta
        self.z_ref  = self.pose.position.z
        if self.th_ref >= 0:
            self.dir =  1
        else:
            self.dir = -1
        
        angle = R - self.th_ref
                                
        if angle > R:
            angle = pi - angle
        
        if   self.th_ref > radians( 10):
            self.tb3.rotate( angle * .9)
        elif self.th_ref < radians(-10):
            self.tb3.rotate(-angle * .97)
        else:
            pass        
        print("\n----- 3_1st rotation finished!\n") #########################
        
        dist1 = abs(self.z_ref * sin(self.th_ref) * 1.125)
        self.tb3.straight(dist1)
        print("\n----- 4_move to front of marker end!\n") ###################
        
        if   self.th_ref >  radians(10):
            self.tb3.rotate(-R * 0.875)
        elif self.th_ref < -radians(10):
            self.tb3.rotate( R)
        else:
            pass        
        print("\n----- 5_2nd rotation finished!\n") #########################
        
        while self.pose.position.x < -0.0025 or self.pose.position.x >  0.0025:
            if   self.pose.position.x < -0.0025:
                self.tw.angular.z =  0.075 * ANG_SPEED
            elif self.pose.position.x >  0.0025:
                self.tw.angular.z = -0.075 * ANG_SPEED
            else:
                self.tw.angular.z =  0.0
                
            self.pub_tw.publish(self.tw)                
            rclpy.spin_once(self, timeout_sec=0.02)
            
        dist2 = self.pose.position.z - 0.11
        self.tb3.straight(dist2)
        print("\n----- 6_arrived lifting position!\n")

        self.pub_lift_msg("lift_up")
        duration = self.cnt_sec + 10
        
        while self.cnt_sec < duration: 
            print(duration - self.cnt_sec)               
            rclpy.spin_once(self, timeout_sec=1.0)
        print("\n----- 7_finished loading!\n") ############################     
        
        self.tb3.straight(-dist2)
        self.tb3.rotate(R * self.dir)
        self.tb3.straight(-dist1)
        print("\n----- 8_arrived starting point!\n") ######################
       
        os.system("ros2 run project1 get_color")
        sys.exit(1)
        rclpy.spin(self)

def main(args=None):

    rclpy.init(args=args)
    node = TrackMarker()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

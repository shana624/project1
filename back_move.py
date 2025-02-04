import rclpy, os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from project1.move_tb3 import MoveTB3
from math import radians, degrees, sqrt, atan2
from geometry_msgs.msg import Twist

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType



class BackMove(Node):

 def __init__(self):
        
        super().__init__('back_move')
        qos_profile = QoSProfile(depth=10)
        self.tb3 = MoveTB3()

        self.declare_parameter('back', 'stop')

        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
       

def main(args=None):
    rclpy.init(args=args)
    node = BackMove()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec = 0.1)

            back = node.get_parameter('back').value  # 파라미터 읽기
            #print(back)
            if back == 'go':
                node.get_logger().info("back_move is 'back'.")
                print('moving start') 
                node.tb3.straight(-0.15)
                os.system('ros2 run project1 go_home')
                node.destroy_node()
                rclpy.shutdown()    
                
            else:
                node.get_logger().info("Waiting for 'back' to be 'go'.")
                rclpy.spin_once(node, timeout_sec=1.0)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()
    

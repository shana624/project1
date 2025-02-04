import rclpy
import sys
import os
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import select

class RegParams(Node):
    def __init__(self):
        super().__init__('reg_params')

        # Declare parameters with default values
        self.declare_parameter('arrive_1', 'yet')
        self.declare_parameter('color', None)
        self.declare_parameter('go_stop', 'stop')
        self.declare_parameter('back', 'stop')

        self.get_logger().info("Parameters declared. Waiting for input... (Press '0' to reset)")

    def reset_parameters(self):
        """Reset parameters to their default values using CLI commands"""
        os.system("ros2 param set /reg_params arrive_1 yet")
        os.system("ros2 param set /reg_params color None")
        os.system("ros2 param set /reg_params go_stop stop")
        os.system("ros2 param set /reg_params back stop")
        self.get_logger().info("🔄 All parameters have been reset to defaults.")

    def wait_for_input(self):
        """Check for keyboard input in a non-blocking manner"""
        while rclpy.ok():
            # Use select to check for input without blocking
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                user_input = sys.stdin.read(1).strip()
                if user_input == '0':
                    self.reset_parameters()
                else:
                    self.get_logger().info(f"Input '{user_input}' ignored. Press '0' to reset.")

def main():
    rclpy.init()
    node = RegParams()

    # Setup executor with multithreading
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.wait_for_input()  # Handle input in the main thread
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
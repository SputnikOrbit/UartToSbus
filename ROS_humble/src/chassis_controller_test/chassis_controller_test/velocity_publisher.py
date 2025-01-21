import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(String, 'chassis_command', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        current_time = time.time() - self.start_time
        vx_setpoint = math.sin(2 * math.pi * current_time / 2)  # 2s period sine wave
        vy_setpoint = 0.0  # Example value, you can change it
        wz_setpoint = 0.0  # Example value, you can change it

        message = f"vx{vx_setpoint:.2f}vy{vy_setpoint:.2f}wz{wz_setpoint:.2f}"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

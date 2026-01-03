import rclpy
from rclpy.node import Node
from a2_rt_interfaces.msg import ClosestObstInfo
import time

class CustomMsgSubscriber(Node):
    def __init__(self):
        super().__init__('custom_msg_subscriber')
        self.subscription = self.create_subscription(ClosestObstInfo, '/custom_obstacle', self.listener_callback, 10)
        self.last_print_time = 0.0

    def listener_callback(self, msg):
        if time.time() - self.last_print_time > 5.0:
            self.get_logger().info(f'Received ClosestObstInfo: Distance={msg.closest_distance:.2f}m, Direction={msg.direction}, Threshold={msg.threshold:.2f}m')
            self.last_print_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    custom_subscriber = CustomMsgSubscriber()
    rclpy.spin(custom_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
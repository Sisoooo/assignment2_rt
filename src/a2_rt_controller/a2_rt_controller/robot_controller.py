import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import time

class RobotController(Node):

    def __init__(self):

        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_publisher_ = self.create_publisher(Bool, '/obstacle_detected', 10)

        self.sensor_subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_subscription_ = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.time_subscription_ = self.create_subscription(Float32, '/time_topic', self.time_callback, 10)
        
        self.get_logger().info('Robot Controller Node has been started.')
        
        self.user_linear_x = 0.0
        self.user_angular_z = 0.0
        self.duration = 0.0
        self.reversing = False

    def cmd_vel_callback(self, msg):
        # Update user velocity only if not reversing
        if not self.reversing:
            self.user_linear_x = msg.linear.x
            self.user_angular_z = msg.angular.z

    def time_callback(self, msg):
        self.duration = msg.data

    def scan_callback(self, msg):
        # Filter out invalid readings (inf, nan)
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        obstacle_detected = False
        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < 1.0 and not self.reversing:
                obstacle_detected = True

                stop_flag = Bool()
                stop_flag.data = obstacle_detected
                self.obstacle_publisher_.publish(stop_flag)
                
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m. Reversing...')
                self.reversing = True
                
                reverse_msg = Twist()
                reverse_msg.linear.x = -self.user_linear_x
                reverse_msg.angular.z = -self.user_angular_z
                
                start_time = time.time()
                while time.time() - start_time < self.duration:
                    self.publisher_.publish(reverse_msg)
                    time.sleep(0.1)

                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.publisher_.publish(stop_msg)
                    
                self.reversing = False
        
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



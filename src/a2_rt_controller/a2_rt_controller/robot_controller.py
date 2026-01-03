import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from a2_rt_interfaces.msg import ClosestObstInfo
import time

class RobotController(Node):

    def __init__(self):

        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_publisher_ = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.custom_obstacle_publisher_ = self.create_publisher(ClosestObstInfo, '/custom_obstacle', 10)

        self.sensor_subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_subscription_ = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.time_subscription_ = self.create_subscription(Float32, '/time_topic', self.time_callback, 10)
        
        self.get_logger().info('Robot Controller Node has been started.')
        
        self.user_linear_x = 0.0
        self.user_angular_z = 0.0
        self.duration = 0.0
        self.reversing = False
        self.already_reversed = False
        self.threshold_distance = 0.5
        self.last_print_time = 0.0

    def cmd_vel_callback(self, msg):
        # Update user velocity only if not reversing
        if not self.reversing:
            self.user_linear_x = msg.linear.x
            self.user_angular_z = msg.angular.z

    def time_callback(self, msg):
        if msg.data < self.duration:
            self.already_reversed = False
        self.duration = msg.data

    def scan_callback(self, msg):
        # Filter out invalid readings (inf, nan)
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]

        if valid_ranges:
            min_distance_custom = min(valid_ranges)
            try:
                min_index = msg.ranges.index(min_distance_custom)
                angle = msg.angle_min + min_index * msg.angle_increment
                
                if -0.39 < angle < 0.39:
                    direction = "front"
                elif 0.39 <= angle < 1.18:
                    direction = "front-left"
                elif 1.18 <= angle < 1.96:
                    direction = "left"
                elif 1.96 <= angle < 2.75:
                    direction = "back-left"
                elif angle >= 2.75 or angle <= -2.75:
                    direction = "back"
                elif -2.75 < angle <= -1.96:
                    direction = "back-right"
                elif -1.96 < angle <= -1.18:
                    direction = "right"
                elif -1.18 < angle <= -0.39:
                    direction = "front-right"
                else:
                    direction = "unknown"
            except ValueError:
                direction = "unknown"

            custom_msg = ClosestObstInfo()
            custom_msg.closest_distance = float(min_distance_custom)
            custom_msg.direction = direction
            custom_msg.threshold = float(self.threshold_distance)
            self.custom_obstacle_publisher_.publish(custom_msg)

        obstacle_detected = False

        go_flag = Bool()
        go_flag.data = obstacle_detected
        self.obstacle_publisher_.publish(go_flag)

        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance <= self.threshold_distance and not self.reversing and not self.already_reversed:
                obstacle_detected = True

                stop_flag = Bool()
                stop_flag.data = obstacle_detected
                self.obstacle_publisher_.publish(stop_flag)
                
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m. Reversing...')
                self.reversing = True
                self.already_reversed = True
                
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



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

import time

class TerminalInterface(Node):
    def __init__(self):
        super().__init__('text_interface')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.time_publisher_ = self.create_publisher(Float32, '/time_topic', 10)

        self.subscription = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.obstacle_detected = False

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = TerminalInterface()
    
    try:
        while rclpy.ok():
            print("Interface to control robot, press q at any prompt to quit")
            
            x_input = input("Enter linear velocity (x-axis) as float number (e.g. 1.0): ")
            z_input = input("Enter angular velocity (z-axis) as float number (e.g. 1.0): ")
            
            if x_input.lower() == 'q' or z_input.lower() == 'q':
                rclpy.shutdown()
                break

            try:
                linear_x = float(x_input)
                angular_z = float(z_input)
            except ValueError:
                print("Invalid input for velocities. Please enter float numbers.")
                continue

            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z

            # Publish for 4 seconds or until obstacle detected
            start_time = time.time()
            duration = 4.0
            node.obstacle_detected = False
            
            while time.time() - start_time < duration:
                rclpy.spin_once(node, timeout_sec=0) # Check for callbacks
                
                # Publish current elapsed time
                time_msg = Float32()
                time_msg.data = time.time() - start_time
                node.time_publisher_.publish(time_msg)

                if node.obstacle_detected:
                    print("Obstacle detected! Stopping publication.")
                    break
                node.publisher_.publish(msg)
                time.sleep(0.1) 
                
            # Stop the robot
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            node.publisher_.publish(stop_msg)
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


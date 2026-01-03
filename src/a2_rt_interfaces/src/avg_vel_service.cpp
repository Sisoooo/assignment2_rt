#include "rclcpp/rclcpp.hpp"
#include "a2_rt_interfaces/srv/average_velocities.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

class AvgVelServiceClient : public rclcpp::Node{
public:
  AvgVelServiceClient() : Node("avg_vel_service_client"), count_(0), receiving_input_(false)
  {
    client_ = this->create_client<a2_rt_interfaces::srv::AverageVelocities>("average_velocities");
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&AvgVelServiceClient::topic_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "AvgVelServiceClient started. Waiting for inputs...");
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    bool is_moving = (msg->linear.x != 0.0 || msg->angular.z != 0.0);

    if (is_moving && !receiving_input_) {
      count_++;
      receiving_input_ = true;
      
      // Check if we reached a multiple of 5
      if (count_ % 5 == 0) {
        call_service();
      }
    } else if (!is_moving) {
      receiving_input_ = false;
    }
  }

  void call_service()
  {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service not available");
      return;
    }

    auto request = std::make_shared<a2_rt_interfaces::srv::AverageVelocities::Request>();
    auto future = client_->async_send_request(request, 
      std::bind(&AvgVelServiceClient::response_callback, this, std::placeholders::_1));
  }

  void response_callback(rclcpp::Client<a2_rt_interfaces::srv::AverageVelocities>::SharedFuture future)
  {
    try {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Inputs: %d | Avg Linear X: %.2f, Avg Angular Z: %.2f", 
                  count_, result->avg_linear_x, result->avg_angular_z);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }

  rclcpp::Client<a2_rt_interfaces::srv::AverageVelocities>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  int count_;
  bool receiving_input_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvgVelServiceClient>());
  rclcpp::shutdown();
  return 0;
}
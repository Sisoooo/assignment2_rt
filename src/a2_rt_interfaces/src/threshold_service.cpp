#include "rclcpp/rclcpp.hpp"
#include "a2_rt_interfaces/srv/change_threshold.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("threshold_service_client");
  auto client = node->create_client<a2_rt_interfaces::srv::ChangeThreshold>("change_threshold");

  while (rclcpp::ok()) {
    // Wait for service to be available
    if (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
      continue;
    }

    char decision;
    std::cout << "Do you want to change the threshold? (y/n): ";
    std::cin >> decision;

    auto request = std::make_shared<a2_rt_interfaces::srv::ChangeThreshold::Request>();

    if (decision == 'y' || decision == 'Y') {
      float new_threshold;
      std::cout << "Enter new threshold value: ";
      std::cin >> new_threshold;
      
      request->decision = true;
      request->new_threshold = new_threshold;
    } else {
      std::cout << "Threshold not changed." << std::endl;
      // Wait 60 seconds before asking again
      std::this_thread::sleep_for(60s);
      continue;
    }

    auto result_future = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Result: %s", result_future.get()->result.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    }

    // Wait 60 seconds before asking again
    std::this_thread::sleep_for(60s);
  }

  rclcpp::shutdown();
  return 0;
}

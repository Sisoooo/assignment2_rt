#include "rclcpp/rclcpp.hpp"
#include "a2_rt_interfaces/srv/change_threshold.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("change_point_client");
    auto client = node->create_client<a2_rt_interfaces::srv::FixedPoint>("change_point");

    while(rclcpp::ok()){
        // Wait for service to be available
        if (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
            }
            RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
            continue;
        }
    }

    char decision;
    std::cout << "Do you want to change the fixed point? (y/n): ";
    std::cin >> decision;

    auto request = std::make_shared<a2_rt_interfaces::srv::FixedPoint::Request>();
    if (decision == 'y' || decision == 'Y') {
        float new_x, new_y;
        std::cout << "Enter new fixed point X value: ";
        std::cin >> new_x;
        std::cout << "Enter new fixed point Y value: ";
        std::cin >> new_y;

        request->decision = true;
        request->new_x = new_x;
        request->new_y = new_y;
    } else {
        std::cout << "Fixed point not changed." << std::endl;
        // Wait 30 seconds before asking again
        std::this_thread::sleep_for(30s);
        continue;
    }

}


/**
 * @file add_two_ints_server.cpp
 * @author Kautilya Reddy Chappidi
 * @brief ROS2 Node with a minimal server that provides a service.
 * @version 0.2
 * @date 2023-11-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"
#include <memory>

/**
 * @brief Adds two integers and stores the result in the response.
 * 
 * @param request The request containing the two integers to be added.
 * @param response The response where the sum will be stored.
 */
void add(const std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<tutorial_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<tutorial_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

/**
 * @file add_three_ints_server.cpp
 * @author Kautilya Reddy Chappidi
 * @brief ROS2 Node with a minimal server that provides a service.
 * @version 0.1
 * @date 2023-11-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"                                         // CHANGE

#include <memory>

/**
 * @brief Adds three integers and stores the result in the response.
 * 
 * @param request The request containing three integers.
 * @param response The response to store the sum of the three integers.
 */
void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
          std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
{
  response->sum = request->a + request->b + request->c;                                      // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  // CHANGE
                request->a, request->b, request->c);                                         // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   // CHANGE

  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               // CHANGE
    node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
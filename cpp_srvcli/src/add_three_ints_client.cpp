/**
 * @file add_three_ints_client.cpp
 * @author Kautilya Reddy Chappidi
 * @brief ROS2 Node with a minimal client that calls a service.
 * @version 0.1
 * @date 2023-11-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

/**
 * @brief Main function of the add_three_ints_client node.
 *
 * This function initializes the ROS 2 node, creates a client to the "add_three_ints" service,
 * sends a request to the service with three integer arguments, waits for the response,
 * and prints the sum of the three integers if the service call is successful.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return 0 if the program exits successfully, 1 otherwise.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Invalid number of arguments. Exiting.");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");
  rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");

  auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Sending a request to add_three_ints service.");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sum: " << result_future.get()->sum);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
  }

  rclcpp::shutdown();
  return 0;
}
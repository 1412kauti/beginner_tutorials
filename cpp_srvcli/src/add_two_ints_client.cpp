/**
 * @file add_two_ints_client.cpp
 * @author Kautilya Reddy Chappidi
 * @brief ROS2 Node with a minimal client that calls a service.
 * @version 0.2
 * @date 2023-11-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

/**
 * @brief The main function of the add_two_ints_client program.
 *
 * This function initializes the ROS 2 node, creates a client to the "add_two_ints" service,
 * sends a request to the service with two integers provided as command-line arguments,
 * waits for the service to become available, and then prints the sum received in the response.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line argument strings.
 * @return int The exit status of the program.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<tutorial_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<tutorial_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}

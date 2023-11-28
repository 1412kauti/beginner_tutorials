/**
 * @file add_two_ints_server.cpp
 * @author Kautilya Reddy Chappidi
 * @brief ROS2 Node with a minimal server that provides a service.
 * @version 0.1
 * @date 2023-11-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * @brief A class representing a minimal parameter node.
 */
class MinimalParam : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for MinimalParam.
   */
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  /**
   * @brief Timer callback function.
   */
  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
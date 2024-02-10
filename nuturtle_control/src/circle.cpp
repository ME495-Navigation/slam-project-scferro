/// \file circle.cpp
/// \brief Commands the turtlebot to drive in a circle
///
/// PARAMETERS:
///     angular_velocity (double): the angular velocity of the robot (m)
///     radius (double): the turning radius for the robot (m)
///     frequency (int): the publishing frequency (Hz)
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): velocity commands for the robot
/// SERVERS:
///     reverse (std_srvs::srv::Empty): reverses the robot along the circular path
///     stop (std_srvs::srv::Empty): stops the robot
///     control (nuturtle_control::srv::Control): starts the robot with a specified angular speed and turning radius

// Used ChatGPT for debugging
// Refer to Citation [5] ChatGPT

#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    // Parameters and default values
    declare_parameter("angular_velocity", 4.0);
    declare_parameter("radius", 0.333);
    declare_parameter("frequency", 100);

    // Define parameter variables
    angular_velocity = get_parameter("angular_velocity").as_double();
    radius = get_parameter("radius").as_double();
    frequency = get_parameter("frequency").as_int();
    linear_velocity = radius * angular_velocity;

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Services
    control_srv = create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv = create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    reverse_srv = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / frequency;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Circle::timer_callback, this));
  }

private:
  // Initialize parameter variables
  double angular_velocity, linear_velocity, radius;
  int frequency;
  int state = 1;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv;

  /// \brief The main timer callback, publishes velocity commands
  void timer_callback()
  {
    geometry_msgs::msg::Twist vel_command;
    if (state == 1) {
      // Create velocity command moving in reverse
      vel_command.linear.x = -linear_velocity;
      vel_command.linear.y = 0.0;
      vel_command.angular.z = angular_velocity;
    }
    // Publish command
    cmd_vel_pub->publish(vel_command);
  }

  /// \brief Stops the robot
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // Set state to STOPPED
    state = 0;
    RCLCPP_INFO(this->get_logger(), "Stopping!.");

    // Publish one cmd_vel command with 0 velocity
    geometry_msgs::msg::Twist vel_command;
    vel_command.linear.x = 0.0;
    vel_command.linear.y = 0.0;
    vel_command.angular.z = 0.0;
    cmd_vel_pub->publish(vel_command);
  }

  /// \brief Reverses the direction of the robot
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    RCLCPP_INFO(this->get_logger(), "Reversing!");
    // Change the direction of the robot
    linear_velocity = -linear_velocity;
    angular_velocity = -angular_velocity;
  }

  /// \brief Set the radius and angular velocity of the robot
  /// \param request The desired radius and angular velocity of the robot
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
    // Extract the angular velocity and radius commands
    angular_velocity = request->velocity;
    radius = request->radius;

    // Update the linear velocity command
    linear_velocity = radius * angular_velocity;
    RCLCPP_INFO(this->get_logger(), "Robot radius and velocity updated.");

    // If stopped, start the robot moving forward
    if (state == 0) {
      state = 1;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Circle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

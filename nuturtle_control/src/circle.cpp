/// \file circle.cpp
/// \brief Commands the turtlebot to drive in a circle
///
/// PARAMETERS:
/// PUBLISHES:
/// SERVERS:
/// CLIENTS:
/// BROADCASTS:

#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/Empty.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  enum class State {
      STOPPED,
      FORWARD,
      REVERSE
  };

  Circle()
  : Node("circle"), state(State::STOPPED)
  {
    // Parameters and default values
    declare_parameter("angular_velocity", 0.1);
    declare_parameter("radius", 0.25);
    declare_parameter("frequency", 100);

    // Define parameter variables
    angular_velocity = get_parameter("angular_velocity").as_double();
    radius = get_parameter("radius").as_double();
    frequency = get_parameter("frequency").as_int();
    linear_velocity = abs(radius * angular_velocity);

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);

    // Services
    control_srv = create_service<nuturtle_control::srv::Control>(
      "~/control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv = create_service<std_srvs::srv::Empty>(
      "~/stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    reverse_srv = create_service<std_srvs::srv::Empty>(
      "~/reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / frequency;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&Circle::timer_callback, this));
  }

private:
  // Initialize parameter variables
  double angular_velocity, linear_velocity, radius;
  uint_32 frequency;
  State state;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr control;

  /// \brief The main timer callback, publishes velocity commands
  void timer_callback()
  {
    geometry_msgs::msg::Twist vel_command;
    if(state == State::REVERSE) {
      // Create velocity command moving in reverse
      vel_command.linear.x = -linear_velocity;
      vel_command.linear.y = 0.0;
      vel_command.angular.z = angular_velocity;
    }
    elif(state == State::FORWARD) {
      // Create velocity command moving forward
      vel_command.linear.x = linear_velocity;
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
    state = State::STOPPED;
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");

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
    // If robot is moving, reverse the direction
    if(state == State::REVERSE) {
        state = State::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Robot direction changed.");
    }
    elif(state == State::FORWARD) {
        state = State::REVERSE;
        RCLCPP_INFO(this->get_logger(), "Robot direction changed.");
    }
  }

  /// \brief Set the radius and angular velocity of the robot
  /// \param request The desired radius and angular velocity of the robot
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
    // Extract the angular velocity and radius commands
    velocity = request.velocity;
    radius = request.radius;
    
    // Update the linear velocity command
    linear_velocity = abs(radius * angular_velocity);
    RCLCPP_INFO(this->get_logger(), "Robot radius and velocity updated.");

    // If stopped, start the robot moving forward
    if(state == State::STOPPED) {
        state = State::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Robot moving forward.");
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

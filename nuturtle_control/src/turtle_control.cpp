/// \file turtle_control.cpp
/// \brief Controls the turtlebot
///
/// PARAMETERS:
/// PUBLISHES:
/// SERVERS:
/// CLIENTS:
/// BROADCASTS:

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Turtle_Control : public rclcpp::Node
{
public:
  Turtle_Control()
  : Node("turtle_control")
  {
    // Parameters and default values
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", 1);
    declare_parameter("motor_cmd_per_rad_sec", 1.0);
    declare_parameter("encoder_ticks_per_rad", 1.0);
    declare_parameter("collision_radius", 1.0);

    // Define parameter variables
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius = get_parameter("collision_radius").as_double();

    // Check if parameters have been defined. if not, throw runtime error
    if (wheel_radius == -1.0 || track_width == -1.0) {
      throw std::runtime_error("Diff drive parameters not defined.");
    }

    // Define other variables
    left_encoder_ticks_prev = 0;
    right_encoder_ticks_prev = 0;
    wheel_speed_cum_error_left = 0.0;
    wheel_speed_cum_error_right = 0.0;
    wheel_speed_error_left_prev = 0.0;
    wheel_speed_error_right_prev = 0.0;
    PID_rate = 100;
    P = 1.0;
    I = 0.0;
    D = 0.0;

    // Initiallize timers
    rclcpp::Time time_start = this->get_clock()->now();
    time_prev_sensor = ((double)(time_start.nanoseconds()) * 0.000000001);

    // Publishers
    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10, std::bind(&Turtle_Control::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data",
      10, std::bind(&Turtle_Control::sensor_data_callback, this, std::placeholders::_1));

    // Main timer
    int cycle_time = 1000.0 / PID_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Turtle_Control::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double wheel_radius, track_width;
  int motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  std::vector<double> target_wheel_speeds = {0.0, 0.0};
  uint64_t left_encoder_ticks, right_encoder_ticks, left_encoder_ticks_prev,
    right_encoder_ticks_prev;
  double time_now_sensor, time_prev_sensor;
  double angle_left_wheel, angle_right_wheel;
  double wheel_speed_left, wheel_speed_right;
  double wheel_speed_error_left, wheel_speed_error_right,
    wheel_speed_error_left_prev, wheel_speed_error_right_prev,
    wheel_speed_cum_error_left, wheel_speed_cum_error_right;
  double P, I, D;
  int PID_rate;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The cmd_vel callback function, publishes motor speed commands based on received twist command
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    // Initialize local variables and messages
    turtlelib::Twist2D twist;

    // Create DiffDrive object
    turtlelib::DiffDrive diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);

    // Extract forward and angular velocities from twist
    twist.x = msg.linear.x;
    twist.omega = msg.angular.z;

    // Find desired wheel speeds using IK
    target_wheel_speeds = diff_drive.inverse_kinematics(twist);
  }

  /// \brief The main timer callback, PID loop for both wheels
  void timer_callback()
  {
    // Initialize local variables and messages
    double left_wheel_accel_error, right_wheel_accel_error;
    int wheel_cmd_left, wheel_cmd_right;
    nuturtlebot_msgs::msg::WheelCommands wheel_commands;

    // Calculate wheel speed errors
    wheel_speed_error_left = target_wheel_speeds[0] - wheel_speed_left;
    wheel_speed_error_right = target_wheel_speeds[1] - wheel_speed_right;
    wheel_speed_cum_error_left += wheel_speed_error_left / PID_rate;
    wheel_speed_cum_error_right += wheel_speed_error_right / PID_rate;
    left_wheel_accel_error = (wheel_speed_error_left - wheel_speed_error_left_prev) / PID_rate;
    right_wheel_accel_error = (wheel_speed_error_right - wheel_speed_error_right_prev) / PID_rate;

    // Calculate wheel commands
    wheel_cmd_left =
      (int)((wheel_speed_error_left * P) + (wheel_speed_cum_error_left * I) +
      (left_wheel_accel_error * D));
    wheel_cmd_right =
      (int)((wheel_speed_error_right * P) + (wheel_speed_cum_error_right * I) +
      (right_wheel_accel_error * D));
    wheel_cmd_left = limit_cmd(wheel_cmd_left);
    wheel_cmd_right = limit_cmd(wheel_cmd_right);

    // Enter wheel commands into message
    wheel_commands.left_velocity = wheel_cmd_left;
    wheel_commands.right_velocity = wheel_cmd_right;

    // Update wheel_speed_error_prev
    wheel_speed_error_left_prev = wheel_speed_error_left;
    wheel_speed_error_right_prev = wheel_speed_error_right;

    // Publish wheel commands for both wheels
    wheel_cmd_pub->publish(wheel_commands);
  }

  /// \brief Limits the range of a wheel_cmd to [-motor_cmd_max, motor_cmd_max]
  int limit_cmd(int wheel_cmd)
  {
    if (wheel_cmd > motor_cmd_max) {
      wheel_cmd = motor_cmd_max;
    } else if (wheel_cmd < -motor_cmd_max) {
      wheel_cmd = -motor_cmd_max;
    }

    return wheel_cmd;
  }

  /// \brief The sensor_data callback function, publishes the current joint_states of the wheels based on received sensor_data
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    // Initialize local variables and messages
    sensor_msgs::msg::JointState wheel_state;

    // Read encoder positions and time from msg and calculate angles
    left_encoder_ticks = msg.left_encoder;
    right_encoder_ticks = msg.right_encoder;
    time_now_sensor = (double)(msg.stamp.sec) + ((double)(msg.stamp.nanosec) * 0.000000001);
    angle_left_wheel = left_encoder_ticks / encoder_ticks_per_rad;      // radians
    angle_right_wheel = right_encoder_ticks / encoder_ticks_per_rad;    // radians

    // Calculate wheel speeds
    wheel_speed_left = ((left_encoder_ticks - left_encoder_ticks_prev) / encoder_ticks_per_rad) /
      (time_now_sensor - time_prev_sensor);
    wheel_speed_right = ((right_encoder_ticks - right_encoder_ticks_prev) / encoder_ticks_per_rad) /
      (time_now_sensor - time_prev_sensor);

    // Add headers to JointStates
    wheel_state.header.stamp = get_clock()->now();
    wheel_state.header.frame_id = {"wheel_left_joint", "wheel_right_joint"};

    // Enter information into joint state messages
    wheel_state.name = {"wheel_left_joint", "wheel_right_joint"};
    wheel_state.position = {angle_left_wheel, angle_right_wheel};
    wheel_state.velocity = {wheel_speed_left, wheel_speed_right};

    // Update time_prev_sensor and encoders prev
    time_prev_sensor = time_now_sensor;
    left_encoder_ticks_prev = left_encoder_ticks;
    right_encoder_ticks_prev = right_encoder_ticks;

    // Publish joint states for both wheels
    joint_states_pub->publish(wheel_state);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Turtle_Control>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

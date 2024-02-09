/// \file odometry.cpp
/// \brief Tracks changes in the robot position based on wheel encoder data
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
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Parameters and default values
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_radius", 1.0);
    declare_parameter("track_width", 1.0);
    declare_parameter("motor_cmd_max", 1.0);
    declare_parameter("motor_cmd_per_rad_sec", 1.0);
    declare_parameter("encoder_ticks_per_rad", 1);
    declare_parameter("collision_radius", 1.0);

    // Define parameter variables
    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    wheel_radius = get_parameter("rate").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_int();
    collision_radius = get_parameter("collision_radius").as_double();

    // Define other variables
    loop_rate = 10;
    left_wheel_angle = 0.0;
    right_wheel_angle = 0.0;
    left_wheel_speed = 0.0;
    right_wheel_speed = 0.0;
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);

    // Create diff_drive
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);

    // Subscribers
    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_states",
      10, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

    // Services
    initial_pose_srv = create_service<nuturtle_control::srv::Pose>(
      "~/initial_pose",
      std::bind(&Odometry::initial_pose_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Odometry::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double wheel_radius;
  double track_width;
  uint32_t motor_cmd_max;
  double motor_cmd_per_rad_sec;
  int encoder_ticks_per_rad;
  double collision_radius;
  double left_wheel_angle, right_wheel_angle, left_wheel_speed, right_wheel_speed;
  std::string body_id, odom_id, wheel_left, wheel_right;
  int loop_rate;
  turtlelib::DiffDrive diff_drive = turtlelib::DiffDrive(1.0, 1.0);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Service<nuturtle_control::srv::Pose>::SharedPtr initial_pose_srv;
  rclcpp::TimerBase::SharedPtr main_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    // Get transform and twist based on wheel positions
    turtlelib::Transform2D body_tf;
    turtlelib::Twist2D body_twist;
    turtlelib::Vector2D position;
    tf2::Quaternion quaternion;

    body_twist = diff_drive.get_twist(left_wheel_angle, right_wheel_angle);
    body_tf = diff_drive.update_state(left_wheel_angle, right_wheel_angle);
    position = body_tf.translation();
    quaternion.setRPY(0, 0, body_tf.rotation());

    // Create odom message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;

    // Add pose to message
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();
    odom_msg.pose.pose.position.x = position.x;
    odom_msg.pose.pose.position.y = position.y;

    // Add Twist to message
    odom_msg.twist.twist.linear.x = body_twist.x;
    odom_msg.twist.twist.linear.y = body_twist.y;
    odom_msg.twist.twist.angular.z = body_twist.omega;

    // Publish odom_msg
    odom_pub->publish(odom_msg);

    // Publish transform from odom to base_link (same position)
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = odom_id;
    tf.child_frame_id = body_id;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    quaternion.setRPY(0, 0, 0);
    tf.transform.rotation.x = quaternion.x();
    tf.transform.rotation.y = quaternion.y();
    tf.transform.rotation.z = quaternion.z();
    tf.transform.rotation.w = quaternion.w();

    tf_broadcaster->sendTransform(tf);
  }

  /// \brief The joint_state callback function, updates the stored wheel position
  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    // Extract new wheel data from message
    left_wheel_angle = msg.position[0];
    right_wheel_angle = msg.position[1];
  }

  /// \brief Reset the turtlebot to a specified position and orientation
  /// \param request The desired x,y position and theta orientation of the robot
  void initial_pose_callback(
    nuturtle_control::srv::Pose::Request::SharedPtr request,
    nuturtle_control::srv::Pose::Response::SharedPtr)
  {
    // Extract desired x, y, theta
    double x = request->x;
    double y = request->y;
    double theta = request->theta;

    // Update diff_drive with new DiffDrive object at specified position
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width, x, y, theta);
    left_wheel_angle = 0.0;
    right_wheel_angle = 0.0;
    left_wheel_speed = 0.0;
    right_wheel_speed = 0.0;
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Odometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/// \file nusim.cpp
/// \brief Simulates the turtlebot in the environment
///
/// PARAMETERS:
///     rate (double): frequency of the timer (Hz)
///     x0 (double): initial x position of the turtlebot (m)
///     y0 (double): initial y position of the turtlebot (m)
///     theta0 (double): initial theta angle of the turtlebot (rad)
///     obstacles/x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles/y (double[]): list of y coordinates of cylindrical obstacles (m)
///     obstacles/r (double): radius of all cylindrical obstacles (m)
///     arena_x_length: X length of rectangular arena (m)
///     arena_y_length: Y length of rectangular arena (m)
/// SUBSCRIBES:
///     ~/red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): the commands for the wheel motors
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
///     ~/walls (visualization_msgs::msg::MarkerArray): marker objects representing walls of arena
///     red/sensor_data (nuturtlebot_msgs::msg::SensorData): encoder data for the simulated robot
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a specified x, y, theta value
/// CLIENTS:
///     none
/// BROADCASTS:
///    nusim/world -> red/base_footprint

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nusim/srv/teleport.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {
    // Parameters and default values
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 6.0);
    declare_parameter("arena_y_length", 6.0);
    declare_parameter("obstacles_x", std::vector<double>{});
    declare_parameter("obstacles_y", std::vector<double>{});
    declare_parameter("obstacles_radius", 0.038);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.8986469);
    declare_parameter("wheel_radius", 0.066);
    declare_parameter("track_width", 0.160);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();
    x0 = get_parameter("x0").as_double();
    y0 = get_parameter("y0").as_double();
    theta0 = get_parameter("theta0").as_double();
    arena_x_length = get_parameter("arena_x_length").as_double();
    arena_y_length = get_parameter("arena_y_length").as_double();
    obstacles_x = get_parameter("obstacles_x").as_double_array();
    obstacles_y = get_parameter("obstacles_y").as_double_array();
    obstacles_radius = get_parameter("obstacles_radius").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();

    // Create diff_drive, initialize wheel speeds
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);
    right_wheel_vel = 0.;
    left_wheel_vel = 0.;

    // Publishers
    timestep_publisher = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obstacle_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    walls_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

    // Subscribers
    wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd",
      10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Services
    reset_server = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    teleport_server = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Nusim::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int timestep = 0;
  double x0, y0, theta0;
  double arena_x_length, arena_y_length;
  double wheel_radius, track_width;
  double right_wheel_vel, left_wheel_vel, motor_cmd_per_rad_sec;
  turtlelib::DiffDrive diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);
  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_radius;
  double x_gt, y_gt, theta_gt;
  int encoder_ticks_per_rad;
  int loop_rate;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;

  /// \brief The main timer callback function, publishes the current timestep, broadcasts groundtruth transform
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped tf_base;
    tf2::Quaternion quat_robot;
    auto message = std_msgs::msg::UInt64();
    double delta_left_wheel, delta_right_wheel, left_wheel_angle, right_wheel_angle;
    std::vector<double> wheel_angles, state;
    turtlelib::Transform2D body_tf;

    // Add the timestep to the message
    message.data = timestep;

    // Publish the current timestep
    timestep_publisher->publish(message);

    // Calculate wheel position change
    delta_left_wheel = left_wheel_vel / loop_rate;
    delta_right_wheel = right_wheel_vel / loop_rate;

    // Find new wheel positions
    wheel_angles = diff_drive.return_wheels();
    left_wheel_angle = wheel_angles[0] + delta_left_wheel;
    right_wheel_angle = wheel_angles[1] + delta_right_wheel;

    // Update diff_drive state
    wheel_angles = diff_drive.return_wheels();
    body_tf = diff_drive.update_state(left_wheel_angle, right_wheel_angle);
    state = diff_drive.return_state();
    x_gt = state[0];
    y_gt = state[1];
    theta_gt = state[2];

    // Create base link gt transforms
    tf_base.header.stamp = get_clock()->now();
    tf_base.header.frame_id = "nusim/world";
    tf_base.child_frame_id = "red/base_footprint";
    tf_base.transform.translation.x = x_gt;
    tf_base.transform.translation.y = y_gt;
    tf_base.transform.translation.z = 0.0;

    quat_robot.setRPY(0, 0, theta_gt);
    tf_base.transform.rotation.x = quat_robot.x();
    tf_base.transform.rotation.y = quat_robot.y();
    tf_base.transform.rotation.z = quat_robot.z();
    tf_base.transform.rotation.w = quat_robot.w();

    // Publish transforms
    tf_broadcaster->sendTransform(tf_base);

    // Publish walls
    publish_walls();

    // Publish obstacles
    publish_obstacles();

    // Publish sensor data
    publish_sensor_data(left_wheel_angle, right_wheel_angle);

    // Increase timestep
    timestep++;
  }

  /// \brief The wheel_cmd callback function, updates wheel speeds and robot ground truth position
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    int left_vel_in, right_vel_in;
    left_vel_in = msg.left_velocity;
    right_vel_in = msg.right_velocity;

    left_wheel_vel = left_vel_in * motor_cmd_per_rad_sec;
    right_wheel_vel = right_vel_in * motor_cmd_per_rad_sec;
  }

  /// \brief Reset the simulation
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // Set timestep to 0
    timestep = 0;

    // Reset position variables
    x_gt = x0;
    y_gt = y0;
    theta_gt = theta0;
  }

  /// \brief Teleport the turtlebot to the specified position
  /// \param request The desired x,y position and theta orientation of the robot
  void teleport_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    x_gt = request->x;
    y_gt = request->y;
    theta_gt = request->theta;
  }

  /// @brief Publish encoder ticks
  void publish_sensor_data(double left_wheel_angle, double right_wheel_angle)
  {
    int left_encoder, right_encoder;
    double left_encoder_double, right_encoder_double;
    nuturtlebot_msgs::msg::SensorData sensor_data;

    // Calculate encoder counts at current state
    left_encoder_double = encoder_ticks_per_rad * left_wheel_angle;
    right_encoder_double = encoder_ticks_per_rad * right_wheel_angle; 
    left_encoder = static_cast<int>(left_encoder_double);
    right_encoder = static_cast<int>(right_encoder_double);

    // Define sensor data message
    sensor_data.stamp = get_clock()->now();
    sensor_data.left_encoder = left_encoder;
    sensor_data.right_encoder = right_encoder;

    // Publish sensor_data
    sensor_data_pub->publish(sensor_data);
  }

  /// @brief Publish wall marker locations
  void publish_walls()
  {
    // Create wall markers
    visualization_msgs::msg::MarkerArray walls;
    visualization_msgs::msg::Marker wall1, wall2, wall3, wall4;
    walls.markers.push_back(wall1);
    walls.markers.push_back(wall2);
    walls.markers.push_back(wall3);
    walls.markers.push_back(wall4);

    for (int i = 0; i < 4; i++) {
      walls.markers.at(i).header.stamp = get_clock()->now();
      walls.markers.at(i).header.frame_id = "nusim/world";
      walls.markers.at(i).id = i;
      walls.markers.at(i).type = 1;     // box
      walls.markers.at(i).action = 0;

      // Set color
      walls.markers.at(i).color.r = 1.0;
      walls.markers.at(i).color.g = 0.0;
      walls.markers.at(i).color.b = 0.0;
      walls.markers.at(i).color.a = 1.0;

      // Set height to 0.25
      walls.markers.at(i).scale.x = 0.0;
      walls.markers.at(i).scale.y = 0.0;
      walls.markers.at(i).scale.z = 0.25;

      // Set height of marker midpoint to 0.25/2
      walls.markers.at(i).pose.position.x = 0.0;
      walls.markers.at(i).pose.position.y = 0.0;
      walls.markers.at(i).pose.position.z = 0.125;
    }

    const auto thiccness = 0.2;

    // Set dimensions and positions of walls
    walls.markers.at(0).scale.x = thiccness;
    walls.markers.at(1).scale.x = arena_x_length + 2 * thiccness;
    walls.markers.at(2).scale.x = thiccness;
    walls.markers.at(3).scale.x = arena_x_length + 2 * thiccness;

    walls.markers.at(0).scale.y = arena_y_length + 2 * thiccness;
    walls.markers.at(1).scale.y = thiccness;
    walls.markers.at(2).scale.y = arena_y_length + 2 * thiccness;
    walls.markers.at(3).scale.y = thiccness;

    walls.markers.at(0).pose.position.x = 0.5 * (arena_x_length + thiccness);
    walls.markers.at(1).pose.position.y = 0.5 * (arena_y_length + thiccness);
    walls.markers.at(2).pose.position.x = -0.5 * (arena_x_length + thiccness);
    walls.markers.at(3).pose.position.y = -0.5 * (arena_y_length + thiccness);

    // Publish wall markers
    walls_publisher->publish(walls);
  }

  /// @brief Publish obstacle marker locations
  void publish_obstacles()
  {
    visualization_msgs::msg::MarkerArray obstacles;
    u_long num_obs = obstacles_x.size();

    for (size_t i = 0; i < num_obs; i++) {
      // Create obstacle marker
      visualization_msgs::msg::Marker obstacle;
      obstacle.header.stamp = get_clock()->now();
      obstacle.header.frame_id = "nusim/world";
      obstacle.id = i;
      obstacle.type = 3;  // cylinder
      obstacle.action = 0;

      // Set color
      obstacle.color.r = 1.0;
      obstacle.color.g = 0.0;
      obstacle.color.b = 0.0;
      obstacle.color.a = 1.0;

      // Set radius and height
      obstacle.scale.x = 2 * obstacles_radius;
      obstacle.scale.y = 2 * obstacles_radius;
      obstacle.scale.z = 0.25;

      // Set obstacle location
      obstacle.pose.position.x = obstacles_x.at(i);
      obstacle.pose.position.y = obstacles_y.at(i);
      obstacle.pose.position.z = 0.125;

      // Add to marker array
      obstacles.markers.push_back(obstacle);
    }

    // Publish obstacle markers
    obstacle_publisher->publish(obstacles);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nusim>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

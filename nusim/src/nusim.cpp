/// \file nusim.cpp
/// \brief Simulates the turtlebot in the environment
///
/// PARAMETERS:
///     rate (double): frequency of the timer (Hz)
///     x0 (double): initial x position of the turtlebot (m)
///     y0 (double): initial y position of the turtlebot (m)
///     theta0 (double): initial theta angle of the turtlebot (rad)
///     obstacles_x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles_y (double[]): list of y coordinates of cylindrical obstacles (m)
///     obstacles_radius (double): radius of all cylindrical obstacles (m)
///     arena_x_length (double): X length of rectangular arena (m)
///     arena_y_length (double): Y length of rectangular arena (m)
///     input_noise (double): the noise variance for moving the robot
///     slip_fraction (double): defines how much the wheels slip
///     max_range (double): maximum simulated lidar range
/// SUBSCRIBES:
///     ~/red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): the commands for the wheel motors
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
///     ~/walls (visualization_msgs::msg::MarkerArray): marker objects representing walls of arena
///     red/sensor_data (nuturtlebot_msgs::msg::SensorData): encoder data for the simulated robot
///     red/path (nav_msgs::msg::Path): the path of the nusim robot
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a specified x, y, theta value
/// CLIENTS:
///     none
/// BROADCASTS:
///    nusim/world -> red/base_footprint

// Used ChatGPT for debugging
// Refer to Citation [5] ChatGPT
// Refer to Citation [6] ChatGPT

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <functional>

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
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {
    // Parameters and default values
    declare_parameter("rate", 200);
    declare_parameter("fake_sensor_rate", 5);
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
    declare_parameter("pose_rate", 1);
    declare_parameter("input_noise", 1.0);
    declare_parameter("slip_fraction", 0.05);
    declare_parameter("max_range", 3.5);
    declare_parameter("min_range", 0.12);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("collision_radius", 0.11);
    declare_parameter("laser_noise", 0.01);
    declare_parameter("laser_samples", 360);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();
    fake_sensor_rate = get_parameter("fake_sensor_rate").as_int();
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
    pose_rate = get_parameter("pose_rate").as_int();
    input_noise = get_parameter("input_noise").as_double();
    slip_fraction = get_parameter("slip_fraction").as_double();
    max_range = get_parameter("max_range").as_double();
    min_range = get_parameter("min_range").as_double();
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    collision_radius = get_parameter("collision_radius").as_double();
    laser_noise = get_parameter("laser_noise").as_double();
    laser_samples = get_parameter("laser_samples").as_int();

    // Create diff_drive, initialize wheel speeds
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);
    right_wheel_vel = 0.;
    left_wheel_vel = 0.;

    // Create noise and slip ranges
    noise_range = std::normal_distribution<>{0, sqrt(input_noise)};
    slip_range = std::uniform_real_distribution<>{-1.0 * slip_fraction, slip_fraction};
    sensor_range = std::normal_distribution<>{0, sqrt(basic_sensor_variance)};
    laser_range = std::normal_distribution<>{0, sqrt(laser_noise)};

    // Publishers
    timestep_publisher = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obstacle_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    walls_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    fake_sensor_pub = create_publisher<visualization_msgs::msg::MarkerArray>("red/fake_sensor", 10);
    laser_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

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

    // Timers
    int cycle_time = 1000.0 / loop_rate;
    int fake_sensor_cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Nusim::timer_callback, this));
    fake_sensor_timer = this->create_wall_timer(
      std::chrono::milliseconds(fake_sensor_cycle_time),
      std::bind(&Nusim::fake_sensor_timer_callback, this));
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
  std::vector<double> obstacles_x, obstacles_y;
  double obstacles_radius;
  double x_gt, y_gt, theta_gt;
  int encoder_ticks_per_rad, laser_samples;
  int loop_rate, pose_rate, fake_sensor_rate;
  double slip_fraction, input_noise, basic_sensor_variance, laser_noise;
  double max_range, min_range;
  double collision_radius;
  nav_msgs::msg::Path nusim_path;
  geometry_msgs::msg::PoseStamped nusim_pose;
  std::normal_distribution<> noise_range, sensor_range, laser_range;
  std::uniform_real_distribution<> slip_range;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
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
    double left_wheel_angle_slip, right_wheel_angle_slip;

    // Add the timestep to the message
    message.data = timestep;

    // Publish the current timestep
    timestep_publisher->publish(message);

    // Add noise to wheel position updates if wheel are moving
    if (left_wheel_vel != 0.) {
      delta_left_wheel = (left_wheel_vel + noise_range(get_random())) / loop_rate;
    } else {
      delta_left_wheel = 0.0;
    }

    if (right_wheel_vel != 0.) {
      delta_right_wheel = (right_wheel_vel + noise_range(get_random())) / loop_rate;
    } else {
      delta_right_wheel = 0.0;
    }

    // Find wheel angles, adjusted with slip
    wheel_angles = diff_drive.return_wheels();
    left_wheel_angle_slip = wheel_angles[0] + (delta_left_wheel * (1 + slip_range(get_random())));
    right_wheel_angle_slip = wheel_angles[1] + (delta_right_wheel * (1 + slip_range(get_random())));

    // Update diff_drive state
    body_tf = diff_drive.update_state(left_wheel_angle_slip, right_wheel_angle_slip);
    state = diff_drive.return_state();
    wheel_angles = diff_drive.return_wheels();
    x_gt = state[0];
    y_gt = state[1];
    theta_gt = state[2];

    // Check for collisions
    check_collisions();

    // Create base link gt transforms
    // Refer to Citation [3] ChatGPT
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
    publish_sensor_data(wheel_angles[0], wheel_angles[1]);

    // Add current pose to path and publish path
    if ((timestep % (loop_rate / pose_rate))==0) {
      update_path();
    }

    // Increase timestep
    timestep++;
  }

  /// @brief Publishes the fake sensor reading
  void fake_sensor_timer_callback()
  {
    // Update the obstacle positions using the fake sensor
    fake_sensor();

    // Publish the lidar markers
    publish_laser();
  }

  /// @brief Publish a LaserScan msg from the simulated robot lidar
  void publish_laser()
  {
    sensor_msgs::msg::LaserScan laser;
    laser.header.stamp = this->get_clock()->now();
    double x_max, y_max, slope, a, b, c;
    std::string frame_id = "red/base_scan";
    std::vector<float> ranges;
    
    // Basic laser msg data
    laser.header.frame_id = frame_id;
    laser.angle_min = 0.0;
    laser.angle_max = 6.28318530718;
    laser.angle_increment = 6.28318530718 / laser_samples;
    laser.time_increment = 0.0;
    laser.scan_time = 0.20066890120506287;
    laser.range_min = min_range;
    laser.range_max = max_range;

    // check ranges and add to array
    for (int n = 0; n < laser_samples; n++) {
      double angle = n * 6.28318530718 / laser_samples;
      double measurement = max_range;

      // Find maximum range coordinates based on laser range
      x_max = x_gt + max_range * cos(angle + theta_gt);
      y_max = y_gt + max_range * sin(angle + theta_gt);
      slope = (y_max - y_gt) / (x_max - x_gt);

      // iterate through obstacles
      for (u_int i = 0; i < obstacles_x.size(); i++) {
        // Find intersection of obstacle circumference and laser ray
        // Set equation for obstacle equal to equation for ray and solve with quadratic formula
        a = 1 + pow(slope, 2);
        b = 2 * ((y_gt - obstacles_y.at(i) - (slope * x_gt)) * slope - obstacles_x.at(i));
        c = obstacles_x.at(i) * obstacles_x.at(i) + pow((y_gt - obstacles_y.at(i) - (slope * x_gt)), 2) - pow(obstacles_radius, 2);

        // Check if there are solutions
        double disc = b * b - 4 * a * c;
        if (disc > 0) {
          // If disc > 0, the ray intersects the obstacle and solutions exist
          double x1 = (-1.0 * b + sqrt(disc)) / (2 * a);
          double x2 = (-1.0 * b - sqrt(disc)) / (2 * a);
          double y1 = slope * (x1 - x_gt) + y_gt;
          double y2 = slope * (x2 - x_gt) + y_gt;
          double dist1 = sqrt(pow(x_gt - x1, 2) + pow(y_gt - y1, 2));
          double dist2 = sqrt(pow(x_gt - x2, 2) + pow(y_gt - y2, 2));

          // Check if intersection point is within valid measurement range
          bool check1 = ((x1 < x_gt && x1 > x_max) || (x1 > x_gt && x1 < x_max)) && ((y1 < y_gt && y1 > y_max) || (y1 > y_gt && y1 < y_max));
          bool check2 = ((x2 < x_gt && x2 > x_max) || (x2 > x_gt && x2 < x_max)) && ((y2 < y_gt && y2 > y_max) || (y2 > y_gt && y2 < y_max));

          // Select valid solution closer to robot
          if (((dist1 < max_range) && (dist1 < dist2)) && check1) {
            measurement = dist1;
          } else if (((dist2 < max_range) && (dist1 >= dist2)) && check2) {
            measurement = dist2;
          }
        }
      }

      // Check for intersection with walls
      // wall 1, RH side 
      double x_w1 = 0.5 * arena_x_length;
      double y_w1 = slope * (x_w1 - x_gt) + y_gt;
      bool check_w1 = (x_gt <= x_w1) && (x_w1 <= x_max);
      double dist_w1 = sqrt(pow(x_gt - x_w1, 2) + pow(y_gt - y_w1, 2));
      if ((dist_w1 < max_range) && (dist_w1 < measurement) && check_w1) {
        measurement = dist_w1;
      }

      // wall 2, top side 
      double y_w2 = 0.5 * arena_y_length;
      double x_w2 = (1.0 / slope) * (y_w2 - y_gt) + x_gt;
      bool check_w2 = (y_gt <= y_w2) && (y_w2 <= y_max);
      double dist_w2 = sqrt(pow(x_gt - x_w2, 2) + pow(y_gt - y_w2, 2));
      if ((dist_w2 < max_range) && (dist_w2 < measurement) && check_w2) {
        measurement = dist_w2;
      }

      // wall 3, LH side 
      double x_w3 = -0.5 * arena_x_length;
      double y_w3 = slope * (x_w3 - x_gt) + y_gt;
      bool check_w3 = (x_gt >= x_w3) && (x_w3 >= x_max);
      double dist_w3 = sqrt(pow(x_gt - x_w3, 2) + pow(y_gt - y_w3, 2));
      if ((dist_w3 < max_range) && (dist_w3 < measurement) && check_w3) {
        measurement = dist_w3;
      }

      // wall 4, bottom side 
      double y_w4 = -0.5 * arena_y_length;
      double x_w4 = (1.0 / slope) * (y_w4 - y_gt) + x_gt;
      bool check_w4 = (y_gt >= y_w4) && (y_w4 >= y_max);
      double dist_w4 = sqrt(pow(x_gt - x_w4, 2) + pow(y_gt - y_w4, 2));
      if ((dist_w4 < max_range) && (dist_w4 < measurement) && check_w4) {
        measurement = dist_w4;
      }

      // add noise to measurements
      measurement += laser_range(get_random());

      // Add range to array
      ranges.push_back(measurement);
    }

    laser.ranges = ranges;
    
    // Publish laser message
    laser_pub->publish(laser);
  }

  /// @brief Publishes the fake sensor reading
  void fake_sensor()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    turtlelib::Transform2D tf_world_sim = turtlelib::Transform2D{{x_gt, y_gt}, theta_gt};
    turtlelib::Transform2D tf_sim_world = tf_world_sim.inv();

    builtin_interfaces::msg::Time array_time = this->get_clock()->now();

    // Iterate through obstacles
    for (u_int i = 0; i < obstacles_x.size(); i++) {
      // Create marker message
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = array_time;
      marker.header.frame_id = "red/base_footprint";
      marker.id = i;
      marker.type = 3;       // cylinder

      // Check marker distance, delete if out of range
      double obstacle_dist = sqrt(pow((x_gt - obstacles_x.at(i)), 2) + pow((y_gt - obstacles_y.at(i)), 2));
      if (obstacle_dist > max_range) {
        marker.action = 2; // delete
      } else {
        marker.action = 0; // add/modify
      }

      // Set color to yellow
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      // Set radius
      marker.scale.x = 2 * obstacles_radius;
      marker.scale.y = 2 * obstacles_radius;
      marker.scale.z = 0.25;

      // Make a tf for the obstacle
      turtlelib::Transform2D tf_world_obs =
        turtlelib::Transform2D{{obstacles_x.at(i), obstacles_y.at(i)},
        0.0};
      
      // get the obstacle location relative to the robot frame
      const auto tf_sim_obs = tf_sim_world * tf_world_obs;
      marker.pose.position.x = tf_sim_obs.translation().x;
      marker.pose.position.y = tf_sim_obs.translation().y;
      
      marker.pose.position.x += sensor_range(get_random());
      marker.pose.position.y += sensor_range(get_random());
      marker.pose.position.z = 0.125;

      // Add to marker array
      marker_array.markers.push_back(marker);
    }

    // Publish marker array
    fake_sensor_pub->publish(marker_array);
  }

  /// \brief Updates the robot path with the current pose and publishes the path
  void update_path()
  {
    // Update ground truth red turtle path
    nusim_path.header.stamp = get_clock()->now();
    nusim_path.header.frame_id = "nusim/world";

    // Create new pose stamped
    nusim_pose.header.stamp = get_clock()->now();
    nusim_pose.header.frame_id = "nusim/world";
    nusim_pose.pose.position.x = x_gt;
    nusim_pose.pose.position.y = y_gt;
    nusim_pose.pose.position.z = 0.0;

    // Add rotation quaternion about Z
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta_gt);
    nusim_pose.pose.orientation.x = quaternion.x();
    nusim_pose.pose.orientation.y = quaternion.y();
    nusim_pose.pose.orientation.z = quaternion.z();
    nusim_pose.pose.orientation.w = quaternion.w();

    // Add pose to path and publish path
    nusim_path.poses.push_back(nusim_pose);
    path_pub->publish(nusim_path);
  }

  /// @brief Check if the robot has collided with an obstacle and update the config if yes
  void check_collisions()
  {
    for (u_int i = 0; i < obstacles_x.size(); i++) {
      double obstacle_dist = sqrt(pow((x_gt - obstacles_x.at(i)), 2) + pow((y_gt - obstacles_y.at(i)), 2));
      
      // If collided, shift robot tangent to obstacle
      if (obstacle_dist < collision_radius + obstacles_radius) {
        const auto ux = (x_gt - obstacles_x.at(i)) / obstacle_dist;
        const auto uy = (y_gt - obstacles_y.at(i)) / obstacle_dist;
        const auto shift = collision_radius + obstacles_radius - obstacle_dist;
        x_gt += shift * ux;
        y_gt += shift * uy;

        // update robot position
        diff_drive.set_state(x_gt, y_gt, theta_gt);
      }
    }
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

  /// \brief Function for selecting random number from a range
  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    return mt;
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
    // Refer to Citation [3] ChatGPT
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

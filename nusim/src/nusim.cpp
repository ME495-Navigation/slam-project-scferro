\file
\brief 

#include <chrono>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nusim/srv/teleport.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class Nusim : public rclcpp::Node
{
public:
    Nusim() : Node("nusim"), timestep_(0)
    {
        // Define parameters and default values
        declare_parameter("rate", 200);
        declare_parameter("x0", 10.0);
        declare_parameter("y0", 10.0);
        declare_parameter("theta0", 0.0);
        declare_parameter("arena_x_length", 0.0);
        declare_parameter("arena_y_length", 0.0);

        // Define parameter variables
        int rate = get_parameter("rate").as_int();
        double x0 = get_parameter("x0").as_double();
        double y0 = get_parameter("y0").as_double();
        double theta0 = get_parameter("theta0").as_double();
        double arena_x_length = get_parameter("arena_x_length").as_double();
        double arena_y_length = get_parameter("arena_y_length").as_double();

        // Initialize groundtruth position variables
        double x_gt, y_gt, theta_gt;

        // Publishers
        timestep_publisher = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        obstacle_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
        walls_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

        // Transform broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Services
        reset_server_ = create_service<std_srvs::srv::Empty>(
            "~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
        teleport_server_ = create_service<nusim::srv::Teleport>(
            "~/teleport", std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Main timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/rate),
            std::bind(&Nusim::timerCallback, this));
    }

private:

    /// \brief The main timer callback function, publishes the current timestep, broadcasts groundtruth transform
    void timerCallback()
    {
        // Publish the current timestep
        auto message = std_msgs::msg::UInt64();
        timestep_publisher->publish(message);

        // Update the timestep
        message.data = timestep_++;

        // Publish gt transforms
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_time;

        tf.header.stamp = get_clock()->now();
        tf.header.frame_id = "nusim/world";
        tf.child_frame_id = "red/base_footprint";
        tf.transform.translation.x = x_gt;
        tf.transform.translation.y = y_gt;
        tf.transform.translation.z = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, theta_gt);
        tf.transform.rotation.x = quaternion.x();
        tf.transform.rotation.y = quaternion.y();
        tf.transform.rotation.z = quaternion.z();
        tf.transform.rotation.w = quaternion.w();

        tf_broadcaster->sendTransform(tf);

        // Publish walls
        publish_walls();
    }

    /// \brief Reset the simulation
    void reset_callback(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        // Set timestep to 0
        timestep_ = 0;

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

    /// @brief Publish wall marker locations
    void publish_walls()
    {
        visualization_msgs::msg::MarkerArray walls;

        visualization_msgs::msg::Marker marker1, marker2, marker3, marker4;
        walls.markers.push_back(marker1);
        walls.markers.push_back(marker2);
        walls.markers.push_back(marker3);
        walls.markers.push_back(marker4);

        for (int i = 0; i < 4; i++) {
        walls.markers.at(i).header.stamp = current_time;
        walls.markers.at(i).header.frame_id = "nusim/world";
        walls.markers.at(i).id = i;
        walls.markers.at(i).type = 1;
        walls.markers.at(i).action = 0;

        // set color
        walls.markers.at(i).color.r = 1.0;
        walls.markers.at(i).color.g = 0.0;
        walls.markers.at(i).color.b = 0.0;
        walls.markers.at(i).color.a = 1.0;

        // set height to 0.25
        walls.markers.at(i).scale.x = 0.0;
        walls.markers.at(i).scale.y = 0.0;
        walls.markers.at(i).scale.z = 0.25;

        // set height of marker midpoint to 0.25/2
        walls.markers.at(i).pose.position.x = 0.0;
        walls.markers.at(i).pose.position.y = 0.0;
        walls.markers.at(i).pose.position.z = 0.125;
        }

        const auto thiccness = 0.2;

        // Set dimensions and positions of walls
        walls.markers.at(0).scale.x = thiccness;
        walls.markers.at(1).scale.x = arena_x_length + 2 * thiccness;
        walls.markers.at(2).scale.x = wall_thickness;
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
        walls_publisher->publish(markers);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nusim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
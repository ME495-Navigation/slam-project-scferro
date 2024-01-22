\file
\brief 

#include <chrono>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/UInt64.hpp"
#include "std_srvs/srv/Empty.hpp"
#include "geometry_msgs/msg/TransformStamped.hpp"
#include "nusim/srv/teleport.hpp"

class Nusim : public rclcpp::Node
{
public:
    Nusim() : Node("nusim"), timestep_(0)
    {
        // Define parameters and default values
        declare_parameter("rate", 200);
        declare_parameter("x0", 0.0);
        declare_parameter("y0", 0.0);
        declare_parameter("theta0", 0.0);

        // Define parameter variables
        int rate = get_parameter("rate").as_int();
        double x0 = get_parameter("x0").as_double();
        double y0 = get_parameter("y0").as_double();
        double theta0 = get_parameter("theta0").as_double();

        // Publishers
        timestep_publisher = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

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

    // Initialize position variables
    double x, y, theta;

    /// \brief The main timer callback function, publishes the current timestep, broadcasts current transform
    void timerCallback()
    {
        // Publish the current timestep
        auto message = std_msgs::msg::UInt64();
        timestep_publisher->publish(message);

        // Update the timestep
        message.data = timestep_++;

        // Publish transforms
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_time;

        tf.header.stamp = get_clock()->now();
        tf.header.frame_id = "nusim/world";
        tf.child_frame_id = "red/base_footprint";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, theta);
        tf.transform.rotation.x = quaternion.x();
        tf.transform.rotation.y = quaternion.y();
        tf.transform.rotation.z = quaternion.z();
        tf.transform.rotation.w = quaternion.w();

        // Send the transformation
        tf_broadcaster->sendTransform(tf);
        
    }

    /// \brief Reset the simulation
    void reset_callback(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        // Set timestep to 0
        timestep_ = 0;

        // Reset position variables
        x = x0;
        y = y0;
        theta = theta0;
    }

    /// \brief Teleport the turtlebot to the specified position
    /// \param request The desired x,y position and theta orientation of the robot
    void teleport_callback(
        nusim::srv::Teleport::Request::SharedPtr request,
        nusim::srv::Teleport::Response::SharedPtr)
    {
        x = request->x;
        y = request->y;
        theta = request->theta;
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
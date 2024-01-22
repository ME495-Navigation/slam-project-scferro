\file
\brief 

#include <chrono>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/UInt64.hpp"
#include "std_srvs/srv/Empty.hpp"
#include "nusim/srv/teleport.hpp"

class Nusim : public rclcpp::Node
{
public:
    Nusim() : Node("nusim"), timestep_(0)
    {
        // Define parameters and default values
        declare_parameter("rate", 200);

        // Publishers
        timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        // Services
        reset_server_ = create_service<std_srvs::srv::Empty>(
            "~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
        teleport_server_ = create_service<nusim::srv::Teleport>(
            "~/teleport", std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Main timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&Nusim::timerCallback, this));
    }

private:
    /// \brief The main timer callback function
    void timerCallback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = timestep_++;
        timestep_publisher_->publish(message);
    }

    /// \brief Reset the simulation
    void reset_callback(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        timestep_ = 0;
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
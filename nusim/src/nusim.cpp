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

class Nusim : public rclcpp::Node {

    public:
        Nusim()
        : ("Nusim"), timestep_(0) {

            // Define parameters and default values
            declare_parameter("rate", 200, "Timer callback frequency [Hz]");
            
        }
        
}
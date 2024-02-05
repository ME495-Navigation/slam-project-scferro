#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include<vector>
#include <stdexcept>


namespace turtlelib {

    DiffDrive::DiffDrive(double radius, double track)
    : phi_right(0.0), phi_left(0.0), x_robot(0.0), y_robot(0.0), theta_robot(0.0), track_width(track), wheel_radius(radius) {}

    DiffDrive::DiffDrive(double radius, double track, double x_pos, double y_pos, double theta)
    : phi_right(0.0), phi_left(0.0), x_robot(x_pos), y_robot(y_pos), theta_robot(theta), track_width(track), wheel_radius(radius) {}

    Transform2D DiffDrive::update_state(double new_phi_right, double new_phi_left) {
        Transform2D tf_out;
        Twist2D twist;
        // create twist, assume change in wheel position happens over unit time
        twist.x = wheel_radius * ((new_phi_right - phi_right) + (new_phi_left - phi_left)) / 2;
        twist.omega = wheel_radius * ((new_phi_right - phi_right) - (new_phi_left - phi_left)) / track_width;
        tf_out = integrate_twist(twist);
        return tf_out;
    }

    std::vector<double> DiffDrive::inverse_kinematics(Twist2D twist) {
        if (twist.y==0.0) {
            double vel_left, vel_right;
            double wheel_circumference;
            wheel_circumference = 2 * wheel_radius * PI;
            vel_left = (twist.x / wheel_circumference) - ((track_width * twist.omega) / (2 * wheel_circumference));
            vel_right = (twist.x / wheel_circumference) + ((track_width * twist.omega) / (2 * wheel_circumference));
            return {vel_left, vel_right};   
        } else {
            throw std::logic_error("The y velocity of the twist must be equal to 0.0.");
        }
    }

}
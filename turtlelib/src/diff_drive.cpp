#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"


namespace turtlelib {

    DiffDrive::DiffDrive(double wheel_radius, double track_width)
    : x_robot(0.0), y_robot(0.0), theta_robot(0.0), phi_right(0.0), phi_left(0.0), track_width(track_width), wheel_radius(wheel_radius) {}

    DiffDrive::DiffDrive(double wheel_radius, double track_width, double x_pos, double y_pos, double theta)
    : x_robot(x_pos), y_robot(y_pos), theta_robot(theta), phi_right(0.0), phi_left(0.0), track_width(track_width), wheel_radius(wheel_radius) {}

    Transform2D update_state(double new_phi_right, double new_phi_left) {

    }

    std::vector<double> inverse_kinematics(Twist2D twist) {

    }

}
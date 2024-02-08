#include <cstdio>
#include <cmath>
#include <iostream>
#include <vector>
#include <stdexcept>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"


namespace turtlelib {

    DiffDrive::DiffDrive(double radius, double track)
    : phi_right(0.0), phi_left(0.0), x_robot(0.0), y_robot(0.0), theta_robot(0.0), track_width(track), wheel_radius(radius) {}

    DiffDrive::DiffDrive(double radius, double track, double x_pos, double y_pos, double theta)
    : phi_right(0.0), phi_left(0.0), x_robot(x_pos), y_robot(y_pos), theta_robot(theta), track_width(track), wheel_radius(radius) {}

    Transform2D DiffDrive::update_state(double new_phi_left, double new_phi_right) {
        Transform2D tf_body, tf_space_prev, tf_space;
        Twist2D twist;
        Vector2D trans;
        twist = DiffDrive::get_twist(new_phi_left, new_phi_right);
        tf_body = integrate_twist(twist);
        tf_space_prev = Transform2D({x_robot, y_robot}, theta_robot);
        tf_space = tf_space_prev * tf_body;
        trans = tf_space.translation();
        x_robot = trans.x;
        y_robot = trans.y;
        theta_robot = tf_space.rotation();
        phi_right = new_phi_right;
        phi_left = new_phi_left;
        return tf_body;
    }

    Twist2D DiffDrive::get_twist(double new_phi_left, double new_phi_right) {
        Twist2D twist;
        // create twist, assume change in wheel position happens over unit time
        twist.x = wheel_radius * ((new_phi_right - phi_right) + (new_phi_left - phi_left)) / 2;
        twist.omega = wheel_radius * ((new_phi_right - phi_right) - (new_phi_left - phi_left)) / track_width;
        return twist;
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

    std::vector<double> DiffDrive::return_state() {
        std::vector<double> state = {x_robot, y_robot, theta_robot};
        return state;
    }

}
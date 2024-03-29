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
    : phi_right(0.0), phi_left(0.0), x_robot(0.0), y_robot(0.0), theta_robot(0.0), track_width(track), wheel_diameter(radius) {}

    DiffDrive::DiffDrive(double radius, double track, double x_pos, double y_pos, double theta)
    : phi_right(0.0), phi_left(0.0), x_robot(x_pos), y_robot(y_pos), theta_robot(theta), track_width(track), wheel_diameter(radius) {}

    Transform2D DiffDrive::update_state(double new_phi_left, double new_phi_right) {
        // Refer to Citation [2] ChatGPT
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
        // Refer to Citation [2] ChatGPT
        Twist2D twist;
        // create twist, assume change in wheel position happens over unit time
        twist.x = wheel_diameter * ((new_phi_right - phi_right) + (new_phi_left - phi_left)) / 4;
        twist.omega = wheel_diameter * ((new_phi_right - phi_right) - (new_phi_left - phi_left)) / (2 * track_width);
        return twist;
    }

    std::vector<double> DiffDrive::inverse_kinematics(Twist2D twist) {
        // Refer to Citation [2] ChatGPT
        if (twist.y==0.0) {
            double vel_left, vel_right;
            vel_left = (twist.x - ((track_width / 2) * twist.omega)) / (wheel_diameter / 2);
            vel_right = (twist.x + ((track_width / 2) * twist.omega)) / (wheel_diameter / 2);
            return {vel_left, vel_right};   
        } else {
            throw std::logic_error("The y velocity of the twist must be equal to 0.0.");
        }
    }

    std::vector<double> DiffDrive::return_state() {
        std::vector<double> state = {x_robot, y_robot, theta_robot};
        return state;
    }

    std::vector<double> DiffDrive::return_wheels() {
        std::vector<double> wheels = {phi_left, phi_right};
        return wheels;
    }

    void DiffDrive::set_state(double x_pos, double y_pos, double theta) {
        x_robot = x_pos;
        y_robot = y_pos;
        theta_robot = theta;
    }

}
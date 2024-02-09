#ifndef TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
/// \file
/// \brief Kinematics for the differential drive turtlebot


#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include<vector>

namespace turtlelib
{
    /// \brief a representation of a differential drive robot
    class DiffDrive
    {
    private:
        /// \brief the angle of the right wheel
        double phi_right;
        /// \brief the angle of the left wheel
        double phi_left;
        /// \brief the robot x position;
        double x_robot;
        /// \brief the robot y position
        double y_robot;
        /// \brief the orientation of the robot
        double theta_robot;
        /// \brief the distance between the wheel centerlines
        double track_width;
        /// \brief the radius of the wheels
        double wheel_radius;
    public:
        /// \brief create a new object at the origin and set parameters
        /// \param wheel_radius the radii of the wheels
        /// \param track_width distance between the centerlines of the two wheels
        explicit DiffDrive(double radius, double track);

        /// \brief create a new object at a specified position and set parameters
        /// \param wheel_radius the radii of the wheels
        /// \param track_width distance between the centerlines of the two wheels
        explicit DiffDrive(double radius, double track, double x_pos, double y_pos, double theta);

        /// \brief update the state of the robot based on new wheel positions
        /// \param new_phi_right the new angle of the right wheel
        /// \param new_phi_left the new angle of the left wheel
        /// \return the body tf to move the robot, also updates the current robot state
        Transform2D update_state(double new_phi_left, double new_phi_right);

        /// \brief get the twist of the robot based on the new wheel positions
        /// \param new_phi_right the new angle of the right wheel
        /// \param new_phi_left the new angle of the left wheel
        /// \return the body twist of the robot
        Twist2D get_twist(double new_phi_left, double new_phi_right);

        /// \brief find the required wheel velocities for a given twist
        /// \param twist the twist to find velocities for
        /// \return the required left and right velocities in a vector 
        std::vector<double> inverse_kinematics(Twist2D twist);

        /// \brief return the current position of the robot
        /// \return the x, y, theta position of the robot
        std::vector<double> return_state();

        /// \brief return the current wheel positions of the robot
        /// \return the left and right wheel position of the robot
        std::vector<double> return_wheels();
    };
}

#endif
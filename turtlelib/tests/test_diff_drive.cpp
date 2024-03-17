#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <sstream>

using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::PI;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::DiffDrive;

// Used ChatGPT for debugging
// Refer to Citation [1] ChatGPT

TEST_CASE("Update state, both wheel rotate forwards (pure translation)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    double phi_right, phi_left;
    phi_right = 4*PI;
    phi_left = 4*PI;
    Transform2D tf;
    tf = diff_drive.update_state(phi_left, phi_right);
    
    Point2D point, new_point;
    point = {0.0, 0.0};
    new_point = tf(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(0.6283185307, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Update state, wheels spin opposite directions (pure rotation)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    double phi_right, phi_left;
    phi_right = 4*PI;
    phi_left = -4*PI;
    Transform2D tf;
    tf = diff_drive.update_state(phi_left, phi_right);
    
    Point2D point, new_point;
    point = {0.0, 0.0};
    new_point = tf(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Update state, wheels spin different speeds (drive in an LH arc)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    double phi_right, phi_left;
    phi_right = 4*PI;
    phi_left = 2*PI;
    Transform2D tf;
    tf = diff_drive.update_state(phi_left, phi_right);
    
    Point2D point, new_point;
    point = {0.0, 0.0};
    new_point = tf(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(0.4408389392, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.1432372542, 1e-5));
}

TEST_CASE("Update state, wheels spin different speeds (drive in an RH arc)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    double phi_right, phi_left;
    phi_right = 2*PI;
    phi_left = 4*PI;
    Transform2D tf;
    tf = diff_drive.update_state(phi_left, phi_right);
    
    Point2D point, new_point;
    point = {0.0, 0.0};
    new_point = tf(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(0.4408389392, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(-0.1432372542, 1e-5));
}

TEST_CASE("Integrate twist, both wheel rotate forwards (pure translation)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    Twist2D twist;
    std::vector<double> wheel_speeds;
    twist.x = 1;
    wheel_speeds = diff_drive.inverse_kinematics(twist);

    REQUIRE_THAT(wheel_speeds[0], Catch::Matchers::WithinAbs(20., 1e-5));
    REQUIRE_THAT(wheel_speeds[1], Catch::Matchers::WithinAbs(20., 1e-5));
}

TEST_CASE("Integrate twist, wheels spin opposite directions (pure rotation)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    Twist2D twist;
    std::vector<double> wheel_speeds;
    twist.omega = PI;
    wheel_speeds = diff_drive.inverse_kinematics(twist);

    REQUIRE_THAT(wheel_speeds[0], Catch::Matchers::WithinAbs(-15.7079632679, 1e-5));
    REQUIRE_THAT(wheel_speeds[1], Catch::Matchers::WithinAbs(15.7079632679, 1e-5));
}

TEST_CASE("Integrate twist, wheels spin different speeds (drive in an LH arc)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    Twist2D twist;
    std::vector<double> wheel_speeds;
    twist.omega = PI;
    twist.x = 1;
    wheel_speeds = diff_drive.inverse_kinematics(twist);

    REQUIRE_THAT(wheel_speeds[0], Catch::Matchers::WithinAbs(4.2920367321, 1e-5));
    REQUIRE_THAT(wheel_speeds[1], Catch::Matchers::WithinAbs(35.7079632679, 1e-5));
}

TEST_CASE("Integrate twist, wheels spin different speeds (drive in an RH arc)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    Twist2D twist;
    std::vector<double> wheel_speeds;
    twist.omega = -PI;
    twist.x = 1;
    wheel_speeds = diff_drive.inverse_kinematics(twist);

    REQUIRE_THAT(wheel_speeds[1], Catch::Matchers::WithinAbs(4.2920367321, 1e-5));
    REQUIRE_THAT(wheel_speeds[0], Catch::Matchers::WithinAbs(35.7079632679, 1e-5));
}

TEST_CASE("Integrate twist, invalid twist (y != 0)", "[DiffDrive]") // Stephen Ferro
{
    DiffDrive diff_drive = DiffDrive(0.1, 0.5);
    Twist2D twist;
    std::vector<double> wheel_speeds;
    twist.omega = -PI;
    twist.y = 1.0;

    REQUIRE_THROWS_AS(diff_drive.inverse_kinematics(twist), std::logic_error);
}
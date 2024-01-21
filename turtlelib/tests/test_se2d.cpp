#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::PI;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::deg2rad;

TEST_CASE("Output stream operator for Twist2D", "[operator<< Twist2D]") // Stephen Ferro 
{
    Twist2D point{2.5, 1.2, 3.7};
    std::ostringstream oss;
    oss << point;

    REQUIRE(oss.str() == "[2.5 1.2 3.7]");
}

TEST_CASE("Input stream operator for Twist2D", "[operator>> Twist2D]") // Stephen Ferro
{
    std::istringstream iss("[2.5 1.2 3.7]");
    Point2D point;

    iss >> point;

    REQUIRE(point.w == 2.5);
    REQUIRE(point.x == 1.2);
    REQUIRE(point.y == 3.7);
}

TEST_CASE("New transform, empty", "[Transform2D]") // Stephen Ferro
{
    Transform2D transform = Transform2D();
    REQUIRE_THAT(transform.trans.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(transform.trans.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(transform.rot, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("New transform, rotation only", "[Transform2D]") // Stephen Ferro
{
    double radians = PI;
    Transform2D transform = Transform2D(radians);
    REQUIRE_THAT(transform.trans.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(transform.trans.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(transform.rot, Catch::Matchers::WithinAbs(PI, 1e-5));
}

TEST_CASE("New transform, translation only", "[Transform2D]") // Stephen Ferro
{
    Vector2D vector = [2.0 2.0];
    Transform2D transform = Transform2D(vector);
    REQUIRE_THAT(transform.trans.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(transform.trans.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(transform.rot, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("New transform, rotation and translation", "[Transform2D]") // Stephen Ferro
{
    double radians = PI;
    Vector2D vector = [2.0 2.0];
    Transform2D transform = Transform2D(vector, radians);
    REQUIRE_THAT(transform.trans.x, Catch::Matchers::WithinAbs(-2.0, 1e-5));
    REQUIRE_THAT(transform.trans.y, Catch::Matchers::WithinAbs(-2.0, 1e-5));
    REQUIRE_THAT(transform.rot, Catch::Matchers::WithinAbs(PI, 1e-5));
}

TEST_CASE("Transform a point", "[Transform2D]") // Stephen Ferro
{
    double radians = 0.0;
    Vector2D vector = [2.0 0.0];
    Transform2D transform = Transform2D(vector, radians);
    Point2D point = [0.0 2.0];
    Point2D new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("Transform a vector", "[Transform2D]") // Stephen Ferro
{
    double radians = 0.0;
    Vector2D vector = [2.0 0.0];
    Transform2D transform = Transform2D(vector, radians);
    Vector2D old_vector = [2.0 0.0];
    Vector2D new_vector = transform(old_vector);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Transform a twist", "[Transform2D]") // Stephen Ferro
{
    double radians = PI/2.0;
    Vector2D vector = [0.0 0.0];
    Transform2D transform = Transform2D(vector, radians);
    Twist2D twist = [0.0 2.0 0.0];
    Twist2D new_twist = transform(twist);
    REQUIRE_THAT(new_twist.w, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_twist.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_twist.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

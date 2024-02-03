#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <sstream>

using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::PI;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::deg2rad;

TEST_CASE("Output stream operator for Twist2D", "[operator<< Twist2D]") // Stephen Ferro 
{
    Twist2D twist{2.5, 1.2, 3.7};
    std::ostringstream oss;
    oss << twist;

    REQUIRE(oss.str() == "[2.5 1.2 3.7]");
}

TEST_CASE("Input stream operator for Twist2D", "[operator>> Twist2D]") // Stephen Ferro
{
    std::istringstream iss("[2.5 1.2 3.7]");
    Twist2D twist;

    iss >> twist;

    REQUIRE(twist.omega == 2.5);
    REQUIRE(twist.x == 1.2);
    REQUIRE(twist.y == 3.7);
}

TEST_CASE("New transform, empty", "[Transform2D]") // Stephen Ferro
{
    Transform2D transform;
    Point2D point;
    Point2D new_point;
    point = {2.0, 2.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("New transform, rotation only", "[Transform2D]") // Stephen Ferro
{
    double radians = PI;
    Transform2D transform(radians);
    Point2D point;
    Point2D new_point;
    point = {2.0, 2.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(-2.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(-2.0, 1e-5));
}

TEST_CASE("New transform, translation only", "[Transform2D]") // Stephen Ferro
{
    Vector2D vector = {2.0, 2.0};
    Transform2D transform = Transform2D(vector);
    Point2D point;
    Point2D new_point;
    point = {2.0, 2.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(4.0, 1e-5));
}

TEST_CASE("New transform, rotation and translation", "[Transform2D]") // Stephen Ferro
{
    double radians = PI;
    Vector2D vector = {2.0, 2.0};
    Transform2D transform = Transform2D(vector, radians);
    Point2D point;
    Point2D new_point;
    point = {2.0, 2.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Transform a vector", "[Transform2D]") // Stephen Ferro
{
    double radians = PI;
    Vector2D vector = {0.0, 2.0};
    Transform2D transform = Transform2D(vector, radians);
    Vector2D vector_tf;
    Vector2D vector_tf_new;
    vector_tf = {2.0, 2.0};
    vector_tf_new = transform(vector_tf);
    REQUIRE_THAT(vector_tf_new.x, Catch::Matchers::WithinAbs(-2.0, 1e-5));
    REQUIRE_THAT(vector_tf_new.y, Catch::Matchers::WithinAbs(-2.0, 1e-5));
}

TEST_CASE("Transform a twist", "[Transform2D]") // Stephen Ferro
{
    double radians = PI/2.0;
    Vector2D vector = {0.0, 0.0};
    Transform2D transform = Transform2D(vector, radians);
    Twist2D twist = {0.0, 2.0, 0.0};
    Twist2D new_twist = transform(twist);
    REQUIRE_THAT(new_twist.omega, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_twist.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_twist.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("Inverse of transform", "[Transform2D]") // Stephen Ferro
{
    double radians = PI;
    Vector2D vector = {5.0, 2.0};
    Transform2D transform = Transform2D(vector, radians);
    Point2D point;
    Point2D new_point;
    Point2D inv_point;
    Transform2D transform_inv = transform.inv();
    point = {0.0, 2.0};
    new_point = transform(point);
    inv_point = transform_inv(new_point);
    REQUIRE_THAT(inv_point.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(inv_point.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("Output stream operator for Transform2D", "[operator<< Transform2D]") // Stephen Ferro 
{
    Transform2D transform{{2.5, 1.2}, PI};
    std::ostringstream oss;
    oss << transform;

    REQUIRE(oss.str() == "deg: 180 x: 2.5 y: 1.2");
}

TEST_CASE("Input stream operator for Transform2D", "[operator>> Transform2D]") // Stephen Ferro
{
    std::istringstream iss("deg: 0 x: 2.5 y: 1.2");
    Transform2D transform;

    iss >> transform;

    Point2D point;
    Point2D new_point;
    point = {0.0, 0.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(2.5, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(1.2, 1e-5));
}

TEST_CASE("Integrate a twist, Translation", "[Transform2D]") // Stephen Ferro
{
    Transform2D transform;
    Twist2D twist = {0.0, 2.0, 0.0};
    transform = integrate_twist(twist);

    Point2D point;
    Point2D new_point;
    point = {1.0, 0.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Integrate a twist, Rotation", "[Transform2D]") // Stephen Ferro
{
    Transform2D transform;
    Twist2D twist = {PI, 0.0, 0.0};
    transform = integrate_twist(twist);

    Point2D point;
    Point2D new_point;
    point = {1.0, 0.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Integrate a twist, Combined", "[Transform2D]") // Stephen Ferro
{
    Transform2D transform;
    Twist2D twist = {PI, 2.0, 0.0};
    transform = integrate_twist(twist);

    Point2D point, new_point;
    point = {1.0, 0.0};
    new_point = transform(point);
    REQUIRE_THAT(new_point.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(new_point.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"

using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::normalize_angle;
using turtlelib::PI;

TEST_CASE("Normalizing angles in radians","[normalize_angle]") // Stephen Ferro
{
    REQUIRE_THAT(normalize_angle(PI), Catch::Matchers::WithinAbs(PI, 1e-5));
    REQUIRE_THAT(normalize_angle(-PI), Catch::Matchers::WithinAbs(PI, 1e-5));
    REQUIRE_THAT(normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(normalize_angle(-PI/4.0), Catch::Matchers::WithinAbs(-PI/4.0, 1e-5));
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), Catch::Matchers::WithinAbs(-PI/2.0, 1e-5));
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), Catch::Matchers::WithinAbs(-PI/2.0, 1e-5));
    REQUIRE_THAT(normalize_angle(2.0*PI), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(normalize_angle(-2.0*PI), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Vector + Point = New Point", "[operator+]") // Stephen Ferro
{
    Point2D point = {2.5, 3.0};
    Vector2D vector = {4.0, -3.0};
    REQUIRE_THAT((point+vector).x, Catch::Matchers::WithinAbs(6.5, 1e-5));
    REQUIRE_THAT((point+vector).y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Point1 - Point2 = New Vector", "[operator-]") // Stephen Ferro 
{
    Point2D head = {5.5, 2.0};
    Point2D tail = {2.0, 1.0};
    REQUIRE_THAT((head-tail).x, Catch::Matchers::WithinAbs(3.5, 1e-5));
    REQUIRE_THAT((head-tail).y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("Output stream operator for Point2D", "[operator<< Point2D]") // Stephen Ferro 
{
    Point2D point{2.5, 3.7};
    std::ostringstream oss;
    oss << point;

    REQUIRE(oss.str() == "[2.5 3.7]");
}

TEST_CASE("Input stream operator for Point2D", "[operator>> Point2D]") // Stephen Ferro
{
    std::istringstream iss("[2.5 3.7]");
    Point2D point;

    iss >> point;

    REQUIRE(point.x == 2.5);
    REQUIRE(point.y == 3.7);
}

TEST_CASE("Input stream operator for Point2D", "[operator>> Point2D]") // Stephen Ferro
{
    std::istringstream iss("2.5 3.7");
    Point2D point;

    iss >> point;

    REQUIRE(point.x == 2.5);
    REQUIRE(point.y == 3.7);
}

TEST_CASE("Output stream operator for Point2D", "[operator<< Vector2D]") // Stephen Ferro
{
    Vector2D vector{2.5, 3.7};
    std::ostringstream oss;
    oss << vector;

    REQUIRE(oss.str() == "[2.5 3.7]");
}

TEST_CASE("Input stream operator for Vector2D", "[operator>> Vector2D]") // Stephen Ferro
{
    std::istringstream iss("[2.5 3.7]");
    Vector2D vector;

    iss >> vector;

    REQUIRE(vector.x == 2.5);
    REQUIRE(vector.y == 3.7);
}

TEST_CASE("Input stream operator for Vector2D", "[operator>> Vector2D]") // Stephen Ferro
{
    std::istringstream iss("2.5 3.7");
    Vector2D vector;

    iss >> vector;

    REQUIRE(vector.x == 2.5);
    REQUIRE(vector.y == 3.7);
}
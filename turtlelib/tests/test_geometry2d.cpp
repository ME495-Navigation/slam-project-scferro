#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include <sstream>

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

TEST_CASE("Vector + Vector = New vector", "[operator+ Vector2D]") // Stephen Ferro
{
    Vector2D vec1, vec2;
    vec1 = {1., 1.};
    vec2 = {2., 3.};
    REQUIRE_THAT((vec1+vec2).x, Catch::Matchers::WithinAbs(3., 1e-5));
    REQUIRE_THAT((vec1+vec2).y, Catch::Matchers::WithinAbs(4., 1e-5));
}

TEST_CASE("Vector - Vector = New vector", "[operator- Vector2D]") // Stephen Ferro
{
    Vector2D vec1, vec2;
    vec1 = {4., 4.};
    vec2 = {2., 3.};
    REQUIRE_THAT((vec1-vec2).x, Catch::Matchers::WithinAbs(2., 1e-5));
    REQUIRE_THAT((vec1-vec2).y, Catch::Matchers::WithinAbs(1., 1e-5));
}

TEST_CASE("Vector * Scalar = New vector", "[operator* Vector2D]") // Stephen Ferro
{
    Vector2D vec;
    double scalar;
    vec = {1., 1.};
    scalar = 2;
    REQUIRE_THAT((vec*scalar).x, Catch::Matchers::WithinAbs(2., 1e-5));
    REQUIRE_THAT((vec*scalar).y, Catch::Matchers::WithinAbs(2., 1e-5));
}

TEST_CASE("Scalar * Vector = New vector", "[operator* Vector2D]") // Stephen Ferro
{
    Vector2D vec;
    double scalar;
    vec = {1., 1.};
    scalar = 3;
    REQUIRE_THAT((scalar*vec).x, Catch::Matchers::WithinAbs(3., 1e-5));
    REQUIRE_THAT((scalar*vec).y, Catch::Matchers::WithinAbs(3., 1e-5));
}

TEST_CASE("dot product", "[dot Vector2D]") // Stephen Ferro
{
    Vector2D vec1, vec2;
    double dotProd;
    vec1 = {1., 1.};
    vec2 = {2., 3.};
    dotProd = dot(vec1, vec2);
    REQUIRE_THAT(dotProd, Catch::Matchers::WithinAbs(5., 1e-5));
}

TEST_CASE("magnitude", "[magnitude Vector2D]") // Stephen Ferro
{
    Vector2D vec;
    double mag;
    vec = {5., 5.};
    mag = magnitude(vec);
    REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(7.07106781187, 1e-5));

}

TEST_CASE("angle", "[angle Vector2D]") // Stephen Ferro
{
    Vector2D vec1, vec2;
    double ang;
    vec1 = {1., 0.};
    vec2 = {0., 1.};
    ang = angle(vec1, vec2);
    REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(PI/2, 1e-5));
}

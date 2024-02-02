#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <sstream>

using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::PI;
using turtlelib::Transform2D;
using turtlelib::Svg;

TEST_CASE("Draw point", "[Svg::drawPoint(Point2D point, std::string color)]") // Stephen Ferro 
{
    Svg svg;
    Point2D point;
    point = {1,1};
    REQUIRE(svg.drawPoint(point, "purple") == "<circle cx='504.000000' cy='432.000000' r='3' fill='purple' />\n");
}

TEST_CASE("Draw vector", "[Svg::drawVector(Point2D point, Vector2D vector, std::string color)]") // Stephen Ferro
{
    Svg svg;
    Point2D point;
    Vector2D vector;
    point = {0,0};
    vector = {-1,1};
    REQUIRE(svg.drawVector(point, vector, "purple") == "<line x1='312.000000' y1='432.000000' x2='408.000000' y2='528.000000' stroke='purple' stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n");
}
#include <fstream>
#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib {

    std::string Svg::drawPoint(Point2D point, std::string color) {
        std::string newContent;
        newContent += "<circle cx='" + std::to_string(point.x) +
                    "' cy='" + std::to_string(point.y) +
                    "' r='3' fill='" + color + "' />\n";
        svgContent += newContent;
        return newContent;
    }

    std::string Svg::drawVector(Point2D point, Vector2D vector, std::string color) {
        std::string newContent;
        svgContent += "<line x1='" + std::to_string(vector.x + point.x) +
                    "' y1='" + std::to_string(vector.y + point.y) +
                    "' x2='" + std::to_string(point.x) +
                    "' y2='" + std::to_string(point.y) +
                    "' stroke='" + color + "' stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
        svgContent += newContent;
        return newContent;
    }

    std::string Svg::drawCoordinateFrame(Point2D origin, Vector2D x_axis) {
        std::string newContent;
        newContent += "<g>\n";
        newContent += drawVector(origin, x_axis, "red");
        Vector2D y_axis;
        y_axis.x = -x_axis.y;
        y_axis.y = x_axis.x;
        newContent += drawVector(origin, y_axis, "green");
        newContent += "</g>\n";
        return newContent;
    }

    int Svg::saveToFile(std::string filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << "<svg width='100' height='100' xmlns='http://www.w3.org/2000/svg'>\n";
            file << svgContent;
            file << "</svg>\n";
            file.close();
        }
        svgContent.clear(); // Clear content after saving to file
        return 0;
    }

}
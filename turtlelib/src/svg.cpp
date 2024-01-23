#include <fstream>
#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib {

    Svg::Svg() {

        int Svg::drawPoint(const Point2D point, const std::string color) {
            svgContent += "<circle cx='" + std::to_string(point.x) +
                        "' cy='" + std::to_string(point.y) +
                        "' r='3' fill='" + color + "' />\n";
            return 0;
        }

        int Svg::drawVector(const Point2D point, const Vector2D vector, const std::string color) {
            svgContent += "<line x1='" + std::to_string(vector.x + point.x) +
                        "' y1='" + std::to_string(vector.y + point.y) +
                        "' x2='" + std::to_string(point.x) +
                        "' y2='" + std::to_string(point.y) +
                        "' stroke='" + color + "' stroke-width="5" marker-start=\"url(#Arrow1Sstart)\" />\n";
            return 0;
        }

        int Svg::drawCoordinateFrame(const Point2D origin, const Vector2D x_axis) {
            svgContent += "<g>\n";
            drawVector(origin, x_axis);
            Vector2D y_axis;
            y_axis.x = -x_axis.y;
            y_axis.y = x_axis.x;
            drawVector(origin, y_axis);
            svgContent += "</g>\n";
            return 0;
        }

        int Svg::saveToFile(const std::string filename) {
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

}
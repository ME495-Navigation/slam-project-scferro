#include <fstream>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib {

    std::string Svg::drawPoint(Point2D point, std::string color) {
        double origin_svg_x, origin_svg_y;// uninitialized variables
        origin_svg_x = 408.0;// should be constexpr
        origin_svg_y = 528.0;
        std::string newContent;
        newContent += "<circle cx='" + std::to_string(point.x * 96.0 + origin_svg_x) +
                    "' cy='" + std::to_string(-point.y * 96.0 + origin_svg_y) +
                    "' r='3' fill='" + color + "' />\n";
        svgContent += newContent;
        return newContent;
    }

    std::string Svg::drawVector(Point2D point, Vector2D vector, std::string color) {
        double origin_svg_x, origin_svg_y; // unitiailzed variables
        origin_svg_x = 408.0;
        origin_svg_y = 528.0;
        std::string newContent;
        svgContent += "<line x1='" + std::to_string((vector.x + point.x) * 96.0 + origin_svg_x) +
                    "' y1='" + std::to_string(-(vector.y + point.y) * 96.0 + origin_svg_y) +
                    "' x2='" + std::to_string(point.x * 96.0 + origin_svg_x) +
                    "' y2='" + std::to_string(-point.y * 96.0 + origin_svg_y) +
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
            file << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\">\n";// xmlns=\"http://www.w3.org/2000/svg\">\n\n";
            file << "<defs>\n";
            file << "   <marker\n";
            file << "       style=\"overflow:visible\"\n";
            file << "       id=\"Arrow1Sstart\"\n";
            file << "       refX=\"0.0\"\n";
            file << "       refY=\"0.0\"\n";
            file << "       orient=\"auto\">\n";
            file << "           <path\n";
            file << "               transform=\"scale(0.2) translate(6,0)\"\n";
            file << "               style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n";
            file << "               d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n";
            file << "               />\n";
            file << "   </marker>\n";
            file << "</defs>\n\n";
            file << svgContent;
            file << "</svg>\n";
            file.close();
        }
        svgContent.clear(); // Clear content after saving to file
        return 0;
    }

}

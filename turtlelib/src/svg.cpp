#include <fstream>
#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib {

    Svg::Svg() {}

    int Svg::drawPoint(Point2D point, str color) {
        std::ostringstream oss;
        oss << "<circle cx=\"" << point.x << "\" cy=\"" << point.y << "\" r=\"2\" stroke=\"" << color << "\" fill=\"" << color << "\" stroke-width=\"1\" />\n";
        elements.push_back(oss.str());
        return 0; // Success
    }

    int Svg::drawVector(Vector2D vector, str color) {
        std::ostringstream oss;
        oss << "<line x1=\"0\" y1=\"0\" x2=\"" << vector.x << "\" y2=\"" << vector.y << "\" stroke=\"" << color << "\" />\n";
        elements.push_back(oss.str());
        return 0; // Success
    }

    int Svg::drawCoordinateFrame() {

    }

    std::string Svg::toString() const {
        std::ostringstream oss;
        oss << "<svg width=\"500\" height=\"500\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        for (const auto& element : elements) {
            oss << "  " << element;
        }
        oss << "</svg>\n";
        return oss.str();
    }

    int Svg::saveToFile(const std::string& filename) const {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << toString();
            file.close();
            return 0; // Success
        } else {
            return 1; // Failure: Unable to open the file for writing
        }
    }

}
#ifndef TURTLELIB_SVG_HPP
#define TURTLELIB_SVG_HPP

#include <fstream>
#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib {
    class Svg {
    public:
        Svg();
        
        int drawPoint(Point2D point, std::string color);

        int drawVector(Point2D point, Vector2D vector, std::string color);

        int drawCoordinateFrame(Point2D origin, Vector2D x_axis);

        int saveToFile(const std::string filename);

    private:
        std::vector svgContent;
        svgContent += "<svg width=\"8.500000in\" height=\"11.000000in\" 
            viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n\n"
        svgContent += "<defs>\n"
        svgContent += "<marker\n"
        svgContent += "style=\"overflow:visible\"\n"
        svgContent += "id=\"Arrow1Sstart\"\n"
        svgContent += "refX=\"0.0\"\n"
        svgContent += "refY=\"0.0\"\n"
        svgContent += "orient=\"auto\">\n"
        svgContent += "<path\n"
        svgContent += "transform=\"scale(0.2) translate(6,0)\"\n"
        svgContent += "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n"
        svgContent += "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n"
        svgContent += "/>\n"
        svgContent += "</marker>\n"
        svgContent += "</defs>\n\n"
    };

}

#endif
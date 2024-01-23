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
    /// \brief a class for representing an SVG file
    class Svg {
    public:

        /// \brief Draw a point in the SVG
        /// \param point the point to be drawn
        /// \param color the color of the point
        /// \return 0 if executes successfully
        std::string drawPoint(Point2D point, std::string color);

        /// \brief Draw a vector in the SVG
        /// \param point the origin/tail of the vector
        /// \param vector the vector to be drawn
        /// \param color the color of the vector
        /// \return 0 if executes successfully
        std::string drawVector(Point2D point, Vector2D vector, std::string color);

        /// \brief Draw a coordinate frame in the SVG
        /// \param origin the origin of the frame
        /// \param x_axis the x_axis of the frame to be drawn
        /// \return 0 if executes successfully
        std::string drawCoordinateFrame(Point2D origin, Vector2D x_axis);

        /// \brief Writes the SVG to a file
        /// \param filename the filename of the SVG to be saved
        /// \return 0 if executes successfully
        int saveToFile(const std::string filename);

    private:
        std::string svgContent;
        /*
        svgContent += "<svg width=\"8.500000in\" height=\"11.000000in\"viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n\n";
        svgContent += "<defs>\n";
        svgContent += "<marker\n";
        svgContent += "style=\"overflow:visible\"\n";
        svgContent += "id=\"Arrow1Sstart\"\n";
        svgContent += "refX=\"0.0\"\n";
        svgContent += "refY=\"0.0\"\n";
        svgContent += "orient=\"auto\">\n";
        svgContent += "<path\n";
        svgContent += "transform=\"scale(0.2) translate(6,0)\"\n";
        svgContent += "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n";
        svgContent += "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n";
        svgContent += "/>\n";
        svgContent += "</marker>\n";
        svgContent += "</defs>\n\n";
        */
    };

}

#endif
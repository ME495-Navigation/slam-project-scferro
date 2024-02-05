#ifndef TURTLELIB_SVG_HPP
#define TURTLELIB_SVG_HPP

#include <fstream>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib {
    /// \brief a class for representing an SVG file
    class Svg {
    public:

        /// \brief Draw a point in the SVG
        /// \param point the point to be drawn
        /// \param color the color of the point
        /// \return 0 if executes successfully
        std::string drawPoint(Point2D point, std::string color);// const std::string & or std::string_view

        /// \brief Draw a vector in the SVG
        /// \param point the origin/tail of the vector
        /// \param vector the vector to be drawn
        /// \param color the color of the vector
        /// \return 0 if executes successfully
        std::string drawVector(Point2D point, Vector2D vector, std::string color); // const std::string & or std::string_view

        /// \brief Draw a coordinate frame in the SVG
        /// \param origin the origin of the frame
        /// \param x_axis the x_axis of the frame to be drawn
        /// \return 0 if executes successfully
        std::string drawCoordinateFrame(Point2D origin, Vector2D x_axis);

        /// \brief Writes the SVG to a file
        /// \param filename the filename of the SVG to be saved
        /// \return 0 if executes successfully
        // return codes is probably not the best idea. throw an exception on error.
        // rather than clearing svg content, let the user do that if they want, and then this could be a const function (what if the user wants to save to multiple files
        int saveToFile(const std::string filename);

    private:
        std::string svgContent;
    };

}

#endif

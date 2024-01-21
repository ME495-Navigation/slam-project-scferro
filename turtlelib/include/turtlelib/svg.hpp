#ifndef TURTLELIB_SVG_HPP
#define TURTLELIB_SVG_HPP

#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib {
    class Svg {
    public:
        Svg();
        
        int drawPoint(Point2D point, str color);

        int drawVector(Vector2D vector, str color);

        int drawCoordinateFrame();

        std::string toString() const;
        int saveToFile(const std::string& filename) const;

    private:
        std::vector<std::string> elements;
    };

}

#endif
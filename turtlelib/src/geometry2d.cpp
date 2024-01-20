#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/geometry2d.hpp"


namespace turtlelib {

    double normalize_angle(double rad) {
        while (rad >= 2*PI) rad += -2*PI;
        while (rad < 0) rad += 2*PI;
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        os << "[" << p.x << " " << p.y << "]"; // Output format: [x y]
        return os;
    }

    std::istream & operator>>(std::istream & is, Point2D & p) {
        const auto first_char = is.peek()
        if (first_char == "[") {
            is.get();
            is >> p.x;
            is >> p.y;
            is.get();
        } else {
            is >> p.x;
            is >> p.y;
        }

        is.ignore(50, "\n")
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        Vector2D newVector;
        newVector.x = head.x - tail.x;
        newVector.y = head.y - tail.y;
        return newVector;
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        Point2D newPoint;
        newPoint.x = tail.x + disp.x;
        newPoint.y = tail.y + disp.y;
        return newPoint;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << "[" << v.x << " " << v.y << "]"; // Output format: [x y]
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        const auto first_char = is.peek()
        if (first_char == "[") {
            is.get();
            is >> v.x;
            is >> v.y;
            is.get();
        } else {
            is >> v.x;
            is >> v.y;
        }

        is.ignore(50, "\n")
        return is;
    }

}
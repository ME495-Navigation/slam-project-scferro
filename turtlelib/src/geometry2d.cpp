#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/geometry2d.hpp"

// Used ChatGPT for debugging
// Refer to Citation [7] ChatGPT

namespace turtlelib {

    double normalize_angle(double rad) {
        while (rad > PI) rad += -2.0*PI;
        while (rad <= -PI) rad += 2.0*PI;
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        os << "[" << p.x << " " << p.y << "]"; // Output format: [x y]
        return os;
    }

    std::istream & operator>>(std::istream & is, Point2D & p) {
        const auto first_char = is.peek();
        if (first_char == '[') {
            is.get();
            is >> p.x;
            is >> p.y;
            is.get();
        } else {
            is >> p.x;
            is >> p.y;
        }

        is.ignore(50, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Circle2D & circle) {
        os << "[" << circle.x << " " << circle.y << " " << circle.rad "]"; // Output format: [x y rad]
        return os;
    }

    std::istream & operator>>(std::istream & is, Circle2D & circle) {
        const auto first_char = is.peek();
        if (first_char == '[') {
            is.get();
            is >> circle.x;
            is >> circle.y;
            is >> circle.rad;
            is.get();
        } else {
            is >> circle.x;
            is >> circle.y;
            is >> circle.rad;
        }

        is.ignore(50, '\n');
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        Vector2D newVector;
        newVector.x = head.x - tail.x;
        newVector.y = head.y - tail.y;
        return newVector;
    }

    Vector2D operator+(const Vector2D & vector1, const Vector2D & vector2) {
        Vector2D newVector;
        newVector.x = vector1.x + vector2.x;
        newVector.y = vector1.y + vector2.y;
        return newVector;
    }

    Vector2D operator-(const Vector2D & vector1, const Vector2D & vector2) {
        Vector2D newVector;
        newVector.x = vector1.x - vector2.x;
        newVector.y = vector1.y - vector2.y;
        return newVector;
    }

    Vector2D operator*(const Vector2D & vector, const double & scalar) {
        Vector2D newVector;
        newVector.x = vector.x * scalar;
        newVector.y = vector.y * scalar;
        return newVector;
    }

    Vector2D operator*(const double & scalar, const Vector2D & vector) {
        Vector2D newVector;
        newVector.x = vector.x * scalar;
        newVector.y = vector.y * scalar;
        return newVector;
    }

    double dot(const Vector2D & vector1, const Vector2D & vector2) {
        double dot_product;
        dot_product = (vector1.x * vector2.x) + (vector1.y * vector2.y);
        return dot_product;
    }

    double magnitude(const Vector2D & vector) {
        double mag;
        mag = sqrt(pow(vector.x, 2) + pow(vector.y, 2));
        return mag;
    }

    double angle(const Vector2D & vector1, const Vector2D & vector2) {
        double theta;
        theta = acos(dot(vector1, vector2) / (magnitude(vector1) * magnitude(vector2)));
        return theta;
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
        const auto first_char = is.peek();
        if (first_char == '[') {
            is.get();
            is >> v.x;
            is >> v.y;
            is.get();
        } else {
            is >> v.x;
            is >> v.y;
        }

        is.ignore(50, '\n');
        return is;
    }

}
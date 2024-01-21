#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"


namespace turtlelib {

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw) {

    }

    std::istream & operator>>(std::istream & is, Twist2D & tw) {

    }

    Transform2D::Transform2D()
    : trans{0.0, 0.0}, rot(0.0) {}

    Transform2D::Transform2D(Vector2D trans)
    : trans(trans), rot(0.0) {}

    Transform2D::Transform2D(double radians)
    : trans{0.0, 0.0}, rot(radians) {}

    Transform2D::Transform2D(Vector2D trans, double radians)
    : trans(trans), rot(radians) {}

    Point2D Transform2D::operator()(Point2D p) const {

    }

    Vector2D Transform2D::operator()(Vector2D v) const {

    }

    Twist2D Transform2D::operator()(Twist2D v) const {

    }

    Transform2D Transform2D::inv() const {

    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {

    }

    Vector2D Transform2D::translation() const {
        return trans
    }

    double Transform2D::rotation() const {
        return rot
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {

    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {

    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {

    }

}
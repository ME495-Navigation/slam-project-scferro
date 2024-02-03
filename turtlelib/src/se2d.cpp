#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"


namespace turtlelib {

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw) {
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]"; // Output format: [x y]
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw) {
        const auto first_char = is.peek();
        if (first_char == '[') {
            is.get();
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
            is.get();
        } else {
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
        }

        is.ignore(50, '\n');
        return is;
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
        return {p.x * cos(rot) - p.y * sin(rot) + trans.x, p.x * sin(rot) + p.y * cos(rot) + trans.y};
    }

    Vector2D Transform2D::operator()(Vector2D v) const {
        double new_x, new_y;
        new_x = v.x * cos(rot) - v.y * sin(rot);
        new_y = v.x * sin(rot) + v.y * cos(rot);
        return {new_x, new_y}; // transformed vector is the same magnitude as original vector, but rotated
    }

    Twist2D Transform2D::operator()(Twist2D v) const {
        return {v.omega, v.omega * trans.y + v.x * cos(rot) - v.y * sin(rot), -v.omega * trans.x + sin(rot) * v.x + cos(rot) * v.y};
    }

    Transform2D Transform2D::inv() const {
        return {{-trans.x * cos(rot) - trans.y * sin(rot), -trans.y * cos(rot) + trans.x * sin(rot)}, -rot};
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        trans.x = cos(rot) * rhs.trans.x - sin(rot) * rhs.trans.y + trans.x;
        trans.y = sin(rot) * rhs.trans.x + cos(rot) * rhs.trans.y + trans.y;
        rot = rot + rhs.rot;
        return *this;
    }

    Vector2D Transform2D::translation() const {
        return trans;
    }

    double Transform2D::rotation() const {
        return rot;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        return os << "deg: " << rad2deg(tf.rot) << " x: " << tf.trans.x << " y: " << tf.trans.y;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        std::string str1, str2, str3;
        Vector2D trans;
        double rot;
        const auto first_char = is.peek();
        if (first_char == 'd') {
            is >> str1;
            is >> rot;
            is >> str2;
            is >> trans.x;
            is >> str3;
            is >> trans.y;
            is.get();
        } else {
            is >> rot;
            is >> trans.x;
            is >> trans.y;
        }

        is.ignore(50, '\n');
        rot = deg2rad(rot);
        tf = Transform2D{trans, rot}; 

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        return lhs*=rhs; 
    }

    Transform2D integrate_twist(const Twist2D & twist) {
        Vector2D vector;
        double omega;
        Transform2D Tsb, Tbb_prime, Tss_prime, Tbs, Ts_prime_b_prime;
        vector.x = twist.x;
        vector.y = twist.y;
        omega = twist.omega;
        if (omega==0.0) {
            Tbb_prime = Transform2D(vector, omega);
        } else {
            Vector2D new_vector;
            new_vector.x = vector.y / omega;
            new_vector.y = -vector.x / omega;
            Tsb = Transform2D(new_vector);
            Tss_prime = Transform2D(omega);
            Tbs = Tsb.inv();
            Ts_prime_b_prime = Tsb;
            Tbb_prime = Tbs * Tss_prime * Ts_prime_b_prime;
        }
        
        return Tbb_prime;
    }

}
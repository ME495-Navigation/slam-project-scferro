#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Svg;

int main() {
    // Enter transforms Tab and Tbc
    Transform2D Tab;
    Transform2D Tbc;
    std::cout << "Enter transform T_{a,b}:\n";
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:\n";
    std::cin >> Tbc;

    // Compute all transformations
    Transform2D Tba = Tab.inv();
    Transform2D Tcb = Tbc.inv();
    Transform2D Tac = Tab * Tbc;
    Transform2D Tca = Tcb * Tba;

    // Display all transformations
    std::cout << "T_{a,b}: " << Tab << "\n";
    std::cout << "T_{b,a}: " << Tba << "\n";
    std::cout << "T_{b,c}: " << Tbc << "\n";
    std::cout << "T_{c,b}: " << Tcb << "\n";
    std::cout << "T_{a,c}: " << Tac << "\n";
    std::cout << "T_{c,a}: " << Tca << "\n";

    // Prompt the user to enter a point pa in Frame {a}
    std::cout << "Enter point p_a:\n";
    Point2D pa;
    std::cin >> pa;

    // Compute pa's location in frames b and c
    Point2D pb = Tba(pa);
    Point2D pc = Tca(pa);

    // Output the locations of all 3 points
    std::cout << "p_a: " << pa << "\n";
    std::cout << "p_b: " << pb << "\n";
    std::cout << "p_c: " << pc << "\n";

    // Origin in each frame
    Point2D oa, ob, oc;
    oa.x = 0.0;
    oa.y = 0.0;
    ob.x = 0.0;
    ob.y = 0.0;
    oc.x = 0.0;
    oc.y = 0.0;

    // Prompt the user to enter a vector vb in frame b
    std::cout << "Enter vector v_b:\n";
    Vector2D vb;
    std::cin >> vb;

    // Output vb expressed in frame a and frame c coordinates
    Vector2D va = Tab(vb);
    Vector2D vc = Tcb(vb);

    // Normalize the vector to form v^b
    double mag;
    mag = sqrt(pow(vb.x, 2) + pow(vb.y, 2));
    Vector2D v_hat_b;
    v_hat_b.x = vb.x / mag;
    v_hat_b.y = vb.y / mag;

    // Output the four vectors
    std::cout << "v_hat_b: " << v_hat_b << "\n";
    std::cout << "v_a: " << va << "\n";
    std::cout << "v_b: " << vb << "\n";
    std::cout << "v_c: " << vc << "\n";

    // Draw points and vectors using SVG
    Svg svg;
    svg.drawPoint(pa, "purple");
    svg.drawPoint(pb, "brown");
    svg.drawPoint(pc, "orange");
    svg.drawVector(oa, va, "purple");
    svg.drawVector(ob, v_hat_b, "brown");
    svg.drawVector(oc, vc, "orange");

    // Output the drawing to /tmp/frames.svg
    svg.saveToFile("/tmp/frames.svg");
    std::cout << "File saved to /tmp/frames.svg.\n";

    return 0;
}
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

int main() {
    // Enter transforms Tab and Tbc
    std::cout << "Enter transform T_{a,b}:\n";
    Transform2D Tab;
    std::cin >> Tab;

    std::cout << "Enter transform T_{b,c}:\n";
    Transform2D Tbc;
    std::cin >> Tbc;

    // Compute transformations
    Transform2D Tba = Tab.inv();
    Transform2D Tcb = Tbc.inv();
    Transform2D Tac = Tba * Tbc;
    Transform2D Tca = Tcb * Tab;

    std::cout << "T_{a,b}: " << Tab << "\n";
    std::cout << "T_{b,a}: " << Tba << "\n";
    std::cout << "T_{b,c}: " << Tbc << "\n";
    std::cout << "T_{c,b}: " << Tcb << "\n";
    std::cout << "T_{a,c}: " << Tac << "\n";
    std::cout << "T_{c,a}: " << Tca << "\n";

    // Prompt the user to enter a point pa in Frame {a}
    std::cout << "Enter point p_a:\n";
    Vector2D pa;
    std::cin >> pa;

    // Compute pa's location in frames b and c
    Vector2D pb = Tba(pa);
    Vector2D pc = Tca(pa);

    // Output the locations of all 3 points
    std::cout << "p_a: " << pa << "\n";
    std::cout << "p_b: " << pb << "\n";
    std::cout << "p_c: " << pc << "\n";

    // Prompt the user to enter a vector vb in frame b
    std::cout << "Enter vector v_b:\n";
    Vector2D vb;
    std::cin >> vb;

    // Normalize the vector to form v^b
    Vector2D v_hat_b = normalize(vb);

    // Output vb expressed in frame a and frame c coordinates
    Vector2D va = Tba(vb);
    Vector2D vc = Tca(vb);

    // Draw points and vectors using SVG
    Svg svg;
    svg.drawPoint(pa, "purple");
    svg.drawPoint(pb, "brown");
    svg.drawPoint(pc, "orange");
    svg.drawVector(v_hat_b, "brown");

    // Output the drawing to /tmp/frames.svg
    svg.saveToFile("/tmp/frames.svg");

    return 0;
}
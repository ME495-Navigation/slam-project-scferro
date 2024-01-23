# Turtlelib Library
## Author
Stephen Ferro
## Description
A library for handling transformations in SE(2) and other turtlebot-related math.
# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
    - Propose three different designs for implementing the ~normalize~ functionality
        1. Create a member function of the Vector2D class that normalizes the vector to a unit vector. 
        2. Create an independent function that takes a Vector2D vector as an argument and returns a new normalized vector. 
        3. Create an independent function that takes a Vector2D vector as an argument and modifies/normalizes it. 

    - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
        1. this method is simple to use, but could be confusing to the user, as they have to track if a vector has been normalized or not.
        2. A pro of this method is that it avoids confusion by requiring the user to create a new normalized vector. However, this is less space efficient than modifying the vector itself. 
        3. A con is that the user needs to track if a vector has been normalized. It is also a more complicated method than option 1 while having similar results. 

    - Which of the methods would you implement and why?
        I would implement option 2. I think it is best to require the normalized vector to be stored as a separate Vector2D to avoid confusion over wether a vector has been normalized or not. 

2. What is the difference between a class and a struct in C++?
    A struct has members that are public by default. A class has members that are private by default, but the can be specified public as well. They are largely used in the same way. A class may be a better option if there are to be many member functions.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
    Vector2D is a struct because it is largely a collection of data and does not contain complex functions. The C++ guidelines recommend using structs for simple collections of data that do not contain complex functions. 

    Transform2D also has private members that should not be modified. Per the C++ core guidelines C.8, you should use a class rather than struct if any member is non-public.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
    This is because of guideline C.46: By default, declare single-argument constructors explicit. Transform2D contains multiple constructors with a single argument. 


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
    Transform2D::inv() is declared const because it returns a new Transform2D and does not modify the original. The *= operator modifies the original object, therefore it is not declared const. This is consistent with the C++ Guidelines.  

# Nusim
## Author:
Stephen Ferro
## Description
This package simulates the turtlebot in a rectangular arena with user specified circular obstacles. A teleport service is offered for moving the robot (akin to picking the robot up in the real world) and a reset service is offered to reset the simulation to it's initial configuration. 
## Launchfile
Use 'ros2 launch nusim nusim.launch.xml' to launch the simulation.
## Parameters
The parameters for the simulation are listed below. These can be changed in the file 'config/basic_world.yaml'.
- rate (double): The frequency of the simulation [Hz]
- x0 (double): The initial X coordinate of the robot [m]
- y0 (double): The initial Y coordinate of the robot [m]
- theta0 (double): The initial theta orientation of the robot [rad]
- arena_x_length (double): The X length of the arena [m]
- arena_y_length (double): The Y length of the arena [m]
- obstacles_x (std::vector<double>): The X coordinates of the obstacles [m]
- obstacles_y (std::vector<double>): The Y coordinates of the obstacles [m]
- obstacles_radius (double): The radius of the circular obstacles [m]
## Screenshot of Simulation
![nusim](images/nusim.png?raw=true "nusim")
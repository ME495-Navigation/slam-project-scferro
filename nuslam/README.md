# NuSLAM

## Author:
Stephen Ferro

## Description
This package contains nodes and launch files for doing SLAM with the Turtlebot 3 robot in an environment with cylindrical obstacles. 

## Launchfiles
These launch files will show the red, blue, and green turtlebots in a simulation environment with cylindrical obstacles. The red robot corresponds to the nusim simulated robot , and the red cylinders are the true obstacle locations (groundtruth). The blue robot corresponds to the robots estimated position based purely on wheel odometry. Finally, the green robot corresponds to the robot's position estimate according to SLAM.

### Launch Using Fake Obstacles in Simulation
Use the command `ros2 launch nuslam slam_fake_obs.launch.xml`. Using this launchfile will publish fake obstacles positions near the ground truth position of the obstacle in the simulation. The published positions have added noise to simulate a real sensor. The lidar data is published and visible, but is not used for SLAM. The positions of the fake obstacles are shown in purple.

### Launch Using LIDAR Data in Simulation
Use the command `ros2 launch nuslam slam_with_lidar.launch.xml`. Using this launchfile will use the lidar data published by the simulation to estimate to positions of the obstacles. The robot then uses SLAM to estimate it's position. The estimated positions of the obstacles are shown in purple.
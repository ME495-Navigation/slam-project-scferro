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

### Launch Using the Real Turtlebot 3 Burger
To run SLAM on the turtlebot, first open an SSH terminal to the turtlebot via a wi-fi network. Next, use the command `ros2 launch nuslam turtlebot_bringup.launch.xml` to launch all the required nodes to run SLAM on the turtlebot. If you would like to visualize the obstacles the turtlebot is detecting, you can run `ros2 launch nuslam pc_bringup.launch.xml` on another PC on the same network. This will open RViz and show the turtlebot and the obstacles it is detecting, but it will not run any of the turtle_control or slam nodes on the client PC.

## Examples of Usage
Below are examples of using the NuSLAM package to do SLAM with the turtlebot 3, both in simulation and with the real robot.

### SLAM Using Fake Obstacles in Simulation
Below is an example of doing SLAM with the turtlebot3 in the RViz NuSim simulation with using the fake sensor to detect obstacles. The yellow obstacles are the fake obstacles published by the sensor in the NuSim node. Though the lidar data is visible, it is not used for detecting obstacles. Overall the robot performs well. We can see the blue odometry robot get separated from the red ground truth when the red robot collides with an obstacles, however the green SLAM robot stay close to the ground truth. 

[![LINK TO YOUTUBE](https://i9.ytimg.com/vi_webp/GZHW5FWFSJA/sddefault.webp?v=65f6f89f&sqp=CLjz268G&rs=AOn4CLALxySIM04DnVomrZTA6pflVG2m7A)](https://www.youtube.com/watch?v=GZHW5FWFSJA)

### SLAM Using LIDAR Data in Simulation
Below is an example of doing SLAM with the turtlebot3 in the RViz NuSim simulation using the data published by the simulated lidar. The purple obstacles are the obstacles detected by the landmarks node. The robot generally is able to detect obstacles accurately and locate itself correctly. The purple obstacles tend to stay very close to the ground truth position, and the green slam obstacles do as well. Towards the end of the run, the robot struggles to detect the left obstacle and initializes a new obstacle at a position where there is none. However, the robot is still able to localize itself accurately. Similarly to above, the green robot stays close to the red robot after colliding with an obstacle.  

[![LINK TO YOUTUBE](https://i9.ytimg.com/vi_webp/GZHW5FWFSJA/sddefault.webp?v=65f6f89f&sqp=CLjz268G&rs=AOn4CLALxySIM04DnVomrZTA6pflVG2m7A)](https://www.youtube.com/watch?v=jyUSyTG43Zc)

### SLAM Using LIDAR Data in on the Real TurtleBot
Here is a demonstration of the the NuSLAM package running on the real turtlebot 3 burger. Similarly to the simulation, the robot is able to fairly accurately detect obstacles and track it's position using the lidar data. It is especially accurate when the robot is within the cluster of obstacles. When it drives further away, it qucikly loses track of it's position. However, once it returns to the cluster, it is able to relocate itself. I even crashed the robot into a fixed obstacle and continued driving to throw off the odometry position estimate, but the robot was still able to relocate itself fairly accurately.

[![LINK TO YOUTUBE](https://i9.ytimg.com/vi_webp/rpW0WM7pRoI/sddefault.webp?v=65f6f89f&sqp=CLjz268G&rs=AOn4CLALxySIM04DnVomrZTA6pflVG2m7A)](https://www.youtube.com/watch?v=rpW0WM7pRoI)

Do to network issues, there was a lot of lag when controlling the robot when I was making this video, so it was difficult to get the robot back to it's exact starting position. The final coordinates of the slam estimate of the robot fluctuated, but they were approximately x ~= -0.3 and y ~= 0.1. This very closely matched the robot's true final position relative to the start point, demonstrating successful localization with SLAM.
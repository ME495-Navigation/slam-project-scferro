# nuturtle_control

## Author:
Stephen Ferro

## Description
This packages handles control of the robot and feedback from it's sensors. It publishes motor commands to turn the wheels based on velocity commands on the `cmd_vel` topic, and it reads from the wheel encoders to estimate to robot's position in the world and update joint states. 

## Launchfiles
To launch the odometry and control nodes, use `ros2 launch nuturtle_control start_robot.launch.xml` This will launch the turtle_control and odometry nodes.

### Optional Parameters
`cmd_src`: This controls where the robot will receive velocity commands from. Use `teleop` for teleop control, `circle` to launch the circle node and test driving in a circle, or none for another method of control. 
`robot`: This determines which robot will be controlled. Use `nusim` to launch the sim, `localhost` to launch the real robot. Use `none` to launch without a robot (e.g. on a remote PC).
`use_rviz`: Determines whether RViz is used. Boolean value. 
`world_config_file`: The filepath of the config file to use for the positions of the obstacles and other simulation environment parameters
`odom_frame`: The odom frame of the robot. Typically, this will just be `odom`
`fake_obstacles`: Determines whether the simulation will publish fake obstacles along with lidar data. Boolean value. 

## Testing on the Real TurtleBot
Below is a video of testing this package. After starting the launch file with `cmd_src:=circle`, the robot starts driving in a circle. By calling the reverse service, the robot begins reversing. Finally, I use teleop control to drive the robot back to the starting position

<iframe width="800" height="450" src="https://www.youtube.com/embed/JjyP8bdBT1g?si=lRUmbJy-m2rO0yMx" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

After driving the robot, I recorded the offset reported by ROS between the blue robot frame and the odom frame. This odometry error when using just the wheel encoders to localize. It is close to the true position of the robot, but there is error. 


```
header:
stamp:
    sec: 1708182453
    nanosec: 239322593
frame_id: odom
child_frame_id: blue/base_footprint
pose:
pose:
    position:
    x: 0.10433022523655823
    y: -0.07044543243778119
    z: 0.0
    orientation:
    x: 0.0
    y: 0.0
    z: 0.1962726224413538
    w: 0.7834138242642745
twist:
twist:
    linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0
---
```
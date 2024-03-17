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

<iframe width="800" height="450" src="https://www.youtube.com/embed/JjyP8bdBT1g?si=lRUmbJy-m2rO0yMx" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
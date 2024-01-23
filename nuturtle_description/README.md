# Nuturtle  Description
## Author:
Stephen Ferro
## Description
This package contains the URDF description of the turtlebot, as well as launchfiles to display the turtlebot in RViz. The turtlebot can be displayed in red, greed, blue, or purple, or all four can be displayed at once using the launchfiles below. 
## Launchfiles
- Use `ros2 launch nuturtle_description load_one.launch.py` to see a single robot in RViz.
- Use `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in RViz.

- `ros2 launch nuturtle_description load_one.launch.py --show-args` outputs:
  ```
Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        true (default): publish joint states, false: no jsp. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_rviz':
        true (default): start rviz, false: don't start rviz. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        purple/red/green/blue: sets color (default purple). Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')
        ```

- `ros2 launch nuturtle_description load_all.launch.xml --show-args` outputs:
  ```
  Arguments (pass arguments as '<name>:=<value>'):
  warnings.warn(

    'use_jsp':
        true (default): publish joint states, false: no jsp. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_rviz':
        true (default): start rviz, false: don't start rviz. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        purple/red/green/blue: sets color (default purple). Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')
        ```
## Screenshot of RViz
![nuturtle_description](images/rviz.png?raw=true "Screenshot of RViz")
## RQT Graph
The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![nuturtle_description](images/rqt_graph.svg?raw=true "RQT Graph")

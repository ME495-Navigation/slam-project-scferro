# Nuturtle  Description
## Author:
Stephen Ferro
## Description
## Launchfiles
- `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
- `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.

- `ros2 launch nuturtle_description load_one.launch.py --show-args` outputs:
  ```
  /opt/ros/iron/lib/python3.10/site-packages/launch/conditions/launch_configuration_equals.py:53: UserWarning: The 'LaunchConfigurationEquals' and 'LaunchConfigurationNotEquals' Conditions are  deprecated. Use the 'EqualsSubstitution' and 'NotEqualsSubstitution' substitutions instead! E.g.:
  IfCondition(
  	EqualsSubstitution(LaunchConfiguration('some_launch_arg'), "some_equality_check")
  )
  warnings.warn(
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
/opt/ros/iron/lib/python3.10/site-packages/launch/conditions/launch_configuration_equals.py:53: UserWarning: The 'LaunchConfigurationEquals' and 'LaunchConfigurationNotEquals' Conditions are  deprecated. Use the 'EqualsSubstitution' and 'NotEqualsSubstitution' substitutions instead! E.g.:
  IfCondition(
  	EqualsSubstitution(LaunchConfiguration('some_launch_arg'), "some_equality_check")
  )
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
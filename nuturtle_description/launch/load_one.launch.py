"""
Shows TurtleBot3 in RViz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp", default_value="true",
                              choices=["true", "false"],
                              description="true (default): use joint_state_publisher, false: no joint states published"),

        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              choices=["true", "false" ],
                              description="true (default): start rviz, otherwise don't start rviz"),

        DeclareLaunchArgument(name="color", default_value="purple",
                              choices=["purple", "red", "green", "blue"],
                              description="sets the color of the robot, options: purple (default), red, green, blue"),

        SetLaunchConfiguration(name="rviz_color", 
                               value=[FindPackageShare("nuturtle_description"),
                                      TextSubstitution(text="/config/basic_"),
                                      LaunchConfiguration("color"),
                                      TextSubstitution(text=".rviz")]
                               ),        

        Node(package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=LaunchConfiguration("color"),
            condition= LaunchConfigurationEquals("use_jsp", "true"),
            ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("color"),
            parameters=[
                {"frame_prefix": PathJoinSubstitution([(LaunchConfiguration('color')), '']),
                 "robot_description" :
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"]), 
                              " color:=", LaunchConfiguration("color")])}
            ],
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=LaunchConfiguration("color"),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare("nuturtle_description"), "config", LaunchConfiguration("rviz_color")])],
            condition=LaunchConfigurationEquals("use_rviz", "true"),
            on_exit=Shutdown()
            )
        ])

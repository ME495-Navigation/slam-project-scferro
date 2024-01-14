"""
Shows TurtleBot3 in RViz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp", default_value="true",
                              description="true (default): use joint_state_publisher, false: no joint states published"),

        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              description="true (default): start rviz, otherwise don't start rviz"),

        DeclareLaunchArgument(name="color", default_value="purple",
                              description="sets the color of the robot, options: purple (default), red, green, blue"),

        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             condition= LaunchConfigurationEquals("use_jsp", "true")
             ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"])])}
            ]
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare("nuturtle_description"), "config", "basic_purple.rviz"])],
            condition=LaunchConfigurationEquals("use_rviz", "true")
            )
        ])

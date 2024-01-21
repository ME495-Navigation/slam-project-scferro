"""Shows TurtleBot3 in RViz."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, SetLaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Declare input argument for using jsp
        DeclareLaunchArgument(name="use_jsp", default_value="true",
                              choices=["true", "false"],
                              description="true (default): publish joint states, false: no jsp"),

        # Declare input argument for using rviz
        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              choices=["true", "false"],
                              description="true (default): start rviz, false: don't start rviz"),

        # Declare input argument for robot color
        DeclareLaunchArgument(name="color", default_value="purple",
                              choices=["purple", "red", "green", "blue"],
                              description="purple/red/green/blue: sets color (default purple)"),

        # Store robot color as LaunchConfiguration
        SetLaunchConfiguration(name="rviz_color",
                               value=[FindPackageShare("nuturtle_description"),
                                      TextSubstitution(text="/config/basic_"),
                                      LaunchConfiguration("color"),
                                      TextSubstitution(text=".rviz")]),

        # Launch joint state publisher
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             namespace=LaunchConfiguration("color"),
             condition=LaunchConfigurationEquals("use_jsp", "true"),),

        # Launch robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("color"),
            parameters=[
                {"frame_prefix": PathJoinSubstitution([(LaunchConfiguration('color')), '']),
                 "robot_description":
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"),
                               "urdf", "turtlebot3_burger.urdf.xacro"]),
                          " color:=", LaunchConfiguration("color")])}
            ],
            ),
        # Launch rviz
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=LaunchConfiguration("color"),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare("nuturtle_description"),
                            "config", LaunchConfiguration("rviz_color")])],
            condition=LaunchConfigurationEquals("use_rviz", "true"),
            on_exit=Shutdown()
            )
        ])

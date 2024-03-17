from launch_catch_ros2 import Catch2IntegrationTestNode, Catch2LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    params_file_path = PathJoinSubstitution(
        [FindPackageShare("nuturtle_description"), "config", "diff_params.yaml"]
    )

    return Catch2LaunchDescription(
        [
            Node(
                package="nuturtle_control",
                executable="odometry",
                parameters=[
                    ParameterFile(params_file_path),
                    {
                        "wheel_left": "wheel_left",
                        "wheel_right": "wheel_right",
                        "body_id": "base_footprint",
                    },
                ],
            ),

            Catch2IntegrationTestNode(
                package="nuturtle_control",
                executable="odometry_test",
                parameters=[ParameterFile(params_file_path), {"test_duration": 5.0}],
            ),

            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
            ),
        ]
    )

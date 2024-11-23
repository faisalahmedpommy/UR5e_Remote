from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(get_package_share_directory("ur5e_moveit"),"config","ur5e.urdf.xacro")

            ]
        ),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [
            {'robot_description':robot_description}
        ]
    )
    joint_state_broadcaster_spawn = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    arm_controller_spawn = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_joints_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_broadcaster_spawn,
            arm_controller_spawn
        ]
    )
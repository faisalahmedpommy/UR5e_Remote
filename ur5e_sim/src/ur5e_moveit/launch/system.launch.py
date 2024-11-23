from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the is_sim argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim = LaunchConfiguration("is_sim")
    
    # Get the package share directory
    ur5e_moveit_dir = get_package_share_directory('ur5e_moveit')
    
    # Build the MoveIt configuration
    moveit_config = MoveItConfigsBuilder("ur5e", package_name="ur5e_moveit")\
        .robot_description(os.path.join(ur5e_moveit_dir, 'config', 'ur5e.urdf.xacro'))\
        .robot_description_semantic(file_path=os.path.join(ur5e_moveit_dir, 'config', 'ur5e.srdf'))\
        .trajectory_execution(file_path=os.path.join(ur5e_moveit_dir, 'config', 'moveit_controllers.yaml'))\
        .to_moveit_configs()

    # Node for the MoveIt move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': is_sim},
            {'publish_robot_description_semantic': True}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Rviz2 node configuration
    rviz_config = os.path.join(ur5e_moveit_dir, "config", "moveit.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]
    )

    # Return the launch description
    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node,
            rviz_node
        ]
    )

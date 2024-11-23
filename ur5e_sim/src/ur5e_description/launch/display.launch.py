import launch
import launch_ros.actions
import os

def generate_launch_description():

    # Define package and file paths
    pkg_path = launch_ros.substitutions.FindPackageShare(package='ur5e_description').find('ur5e_description')
    rviz_config_path = os.path.join(pkg_path, 'config/rviz22.rviz')
    model_path = os.path.join(pkg_path, 'urdf/ur5e2.urdf')

    # Read the robot's URDF file
    with open(model_path, 'r') as infp:
        desc = infp.read()

    # Define the robot description parameter
    params = {'robot_description': desc}

    # Define nodes
    robot_state_pub = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        
    )

    # Return the LaunchDescription
    return launch.LaunchDescription([
        robot_state_pub,
        joint_state_publisher_node,
        rviz,
    ])

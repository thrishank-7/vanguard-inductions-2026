import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to the installed package directory
    pkg_share = get_package_share_directory('first_light')
    
    # Point to your rov.urdf file
    urdf_path = os.path.join(pkg_share, 'urdf', 'rov.urdf')

    # Read the URDF file contents
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1. Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 2. Joint State Publisher GUI Node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # 3. RViz2 Visualizer Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

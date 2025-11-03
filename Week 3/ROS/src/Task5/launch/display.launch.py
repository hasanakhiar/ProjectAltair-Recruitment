import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    pkg_dir = get_package_share_directory('Task5')

    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'simple_bot.urdf')

    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'display.rviz')

    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()



    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )


    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )


    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # 6. Return the LaunchDescription
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
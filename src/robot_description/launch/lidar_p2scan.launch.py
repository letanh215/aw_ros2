import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pc2_config_file = os.path.join(get_package_share_directory("robot_description"), "config", "pc2_config.yaml")

    return LaunchDescription([
        Node(
            package="pointcloud2_assembler",
            executable="pointcloud2_assembler",
            name="pointcloud2_assembler",
            parameters= [pc2_config_file],
            remappings=[
                ("cloud_merged", "/lidars/merged_cloud"),
            ]
        ),
        Node(
            package="pointcloud_to_laserscan",
            name="p2l_node",
            executable="pointcloud_to_laserscan_node",
            parameters=[pc2_config_file],
            remappings=[
                ("cloud_in", "/lidars/merged_cloud"),
                ("scan", "scan")
            ]
        )
    ])
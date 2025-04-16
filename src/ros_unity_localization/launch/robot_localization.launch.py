from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory("ros_unity_localization")

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(package_dir, "config", "ekf.yaml")
        ],
        remappings=[
            # ('/tf', '/utf'), ('/tf_static', '/utf_static'),
            ('/odometry/filtered', '/odom/filtered')
        ]
    )

    return LaunchDescription([
        robot_localization
    ])
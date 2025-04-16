from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    ld = LaunchDescription()
    
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('robot_description'), 'urdf', 'nuway_unity.urdf.xml'),
        description="Absolute path to the urdf file."
    )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]), value_type=str)

    robot_descrption_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {"robot_description" : robot_description}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name="rviz2",
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('robot_description'), 'rviz', 'rviz_configs.rviz')],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/initialpose', 'initialpose'),
            ('/clicked_point', 'clicked_point'),
            ('/scan', 'scan')
        ]
    )
    
    ld.add_action(model_arg)
    ld.add_action(robot_descrption_node)
    # ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz_node)

    return ld
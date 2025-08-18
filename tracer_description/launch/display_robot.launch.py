from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("tracer_description")
    default_model_path = pkg_path + '/urdf/tracer_v1.xacro'
    default_rvizconfig_path = pkg_path + '/rviz/model_display.rviz'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Path to urdf file',
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rvizconfig_path),
        description='Path to rviz config file',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
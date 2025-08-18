from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_path = get_package_share_directory("tracer_description")
    urdf_path = desc_path + '/urdf/tracer_v1.xacro'
    default_rviz_path = desc_path + '/rviz/tracer_launch.rviz'

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='',
        description='Robot name'
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_path),
        description='Path to rviz config file',
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Use Rviz to display robot description',
    )

    use_tf_arg = DeclareLaunchArgument(
        name='use_tf',
        default_value='true',
        description='Use TF broadcast',
    )

    simulation_arg = DeclareLaunchArgument(
        name='use_sim',
        default_value='false',
        description='Simulation mode',
    )

    base_node = Node(
        package="tracer_base",
        executable="tracer_base_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            'simulated_robot': LaunchConfiguration('use_sim'),
        }]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'robot_description': Command([
                'xacro ', urdf_path
            ])
        }],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
    )

    base_transform_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name='tracer_transform_broadcaster',
        arguments=[
            *map(str, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            "/world",  # Parent frame
            '/odom',  # Child frame
        ],
        condition=IfCondition(LaunchConfiguration('use_tf')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        robot_name_arg,
        rvizconfig_arg,
        use_rviz_arg,
        use_tf_arg,
        simulation_arg,
        base_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        base_transform_broadcaster,
        rviz_node,
    ])
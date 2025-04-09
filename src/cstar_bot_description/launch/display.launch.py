from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='cstar_bot_description').find('cstar_bot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'cstar_bot_description.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')

    # # Ensure this path is correctly set
    # gz_spawn_model_launch_source = os.path.join(pkg_share, 'launch', 'gz_spawn_model.launch.py')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
                    #{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    static_transform_publisher_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    static_transform_publisher_links = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_links',
        arguments=['0', '0', '0.127', '0', '0', '0', 'base_link', 'lidar_link']
    )

    static_transform_publisher_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser']
    )

    # static_transform_publisher_imu = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_imu',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # # Replace GzServer with ExecuteProcess (gz sim replaces gzserver)
    # gz_sim = ExecuteProcess(
    #     cmd=["ign", "gazebo", world_path, "-g"],  # '-g' loads GUI
    #     output="screen"
    # )
    # gz_server = GzServer(
    #     world_sdf_file=world_path,
    #     container_name='ros_gz_container',
    #     create_own_container='True',
    #     use_composition='True',
    # )

    # ros_gz_bridge = RosGzBridge(
    #     bridge_name='ros_gz_bridge',
    #     config_file=bridge_config_path,
    #     container_name='ros_gz_container',
    #     create_own_container='False',
    #     use_composition='True',
    # )
    # gz_bridge = ExecuteProcess(
    #     cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--config", bridge_config_path],  # '-g' loads GUI
    #     output="screen"
    # )

    # spawn_entity = ExecuteProcess(
    #     cmd=[
    #         "ros2", "run", "ros_gz_sim", "create",
    #         "-world", "my_world",
    #         "-topic", "/robot_description",
    #         "-name", "cstar_bot"
    #     ],
    #     output="screen"
    # )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        static_transform_publisher_odom,
        static_transform_publisher_links,
        static_transform_publisher_laser,
        #static_transform_publisher_imu, 
        # ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        #gz_sim,
        #gz_bridge,
        #spawn_entity,
        robot_localization_node,
        rviz_node
    ])
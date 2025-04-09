import os
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths and parameters for nav2
    nav2_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
    #custom_map_file = os.path.join('<path_to_your_custom_map_directory>', 'custom_map.yaml')
    #bt_xml = os.path.join(nav2_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    return LaunchDescription([
        # Static transform publisher 1
        # TimerAction(
        #     period=2.0,
        #     actions=[
        #         LogInfo(msg="Launching static_transform_publisher for base_link to laser"),
        #         Node(
        #             package='tf2_ros',
        #             executable='static_transform_publisher',
        #             name='static_transform_publisher_1',
        #             arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'],
        #         )
        #     ]
        # ),
        # # Static transform publisher 2
        # TimerAction(
        #     period=4.0,
        #     actions=[
        #         LogInfo(msg="Launching static_transform_publisher for odom to base_link"),
        #         Node(
        #             package='tf2_ros',
        #             executable='static_transform_publisher',
        #             name='static_transform_publisher_2',
        #             arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        #         )
        #     ]
        # ),
        # Static transform publisher 2
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg="Launching SDF with transforms"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('cstar_bot_description'),
                                'launch',
                                'display.launch.py'
                            )
                        ]
                    ),
                )
            ]
        ),
        # RPLidar node
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="Launching RPLidar node"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('rplidar_ros'),
                                'launch',
                                'rplidar_a1_launch.py'
                            )
                        ]
                    ),
                )
            ]
        ),
        # Laser odometry node
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="Launching laser odometry node"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('rf2o_laser_odometry'),
                                'launch',
                                'rf2o_laser_odometry.launch.py'
                            )
                        ]
                    ),
                )
            ]
        ),
        # # Encoder odometry node
        # TimerAction(
        #     period=10.0,
        #     actions=[
        #         LogInfo(msg="Launching encoder odometry node"),
        #         Node(
        #             package='encoder_ros',
        #             executable='encoder_ros_exe',
        #             name='encoder_ros_node',
        #             output='screen',
        #             parameters=[]
        #         )
        #     ]
        # ),
        # # SLAM Toolbox node
        TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg="Launching SLAM Toolbox node"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('slam_toolbox'),
                                'launch',
                                'online_async_launch.py'
                            )
                        ]
                    ),
                )
            ]
        ),
        #Nav2 nodes
        # TimerAction(
        #     period=18.0,
        #     actions=[
        #         LogInfo(msg="Launching Nav2 nodes"),
        #         Node(
        #             package='nav2_bringup',
        #             executable='navigation_launch.py',
        #             name='nav2_bringup',
        #             output='screen',
        #             parameters=[
        #                 '/home/cstar-capstone/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml'
        #             ],
        #             arguments=[
        #                 '--params_file', '/home/cstar-capstone/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml'
        #             ]
        #             #launch_arguments={
        #                 #'map': custom_map_file,
        #                 #'use_sim_time': 'false',
        #                 #'params_file': params_file,
        #                 #'bt_xml_file': bt_xml
        #             #}.items(),
        #         )
        #     ]
        # ),
        # TimerAction(
        #     period=18.0,
        #     actions=[
        #         LogInfo(msg="Launching Nav2 nodes"),
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        #             ),
        #             #launch_arguments={
        #                 #'map': custom_map_file,
        #                 #'use_sim_time': 'false',
        #                 #'params_file': params_file,
        #                 #'bt_xml_file': bt_xml
        #             #}.items(),
        #         )
        #     ]
        # ),
        TimerAction(
            period=18.0,
            actions=[
                LogInfo(msg="Launching Nav2 nodes"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('nav2_bringup'),
                                'launch',
                                'navigation_launch.py'
                            )
                        ]
                    ),
                )
            ]
        ),
        # TimerAction(
        #     period=18.0,
        #     actions=[
        #         LogInfo(msg="Launching costmap nodes"),
        #         Node(
        #             package='nav2_costmap_2d',
        #             executable='nav2_costmap_2d_markers',
        #             name='costmap_markers_node',
        #             output='screen',
        #             parameters=[],
        #             # remappings=[
        #             #     ('voxel_grid', '/local_costmap/voxel_grid'),
        #             #     ('visualization_marker', '/my_marker')
        #             # ]
        #         )
        #     ]
        # ),
        TimerAction(
            period=22.0,
            actions=[
                LogInfo(msg="Launching motor control"),
                Node(
                    package='motor_control',
                    executable='motor_control_exe',
                    name='motor_control_node',
                    output='screen',
                    parameters=[]
                )
            ]
        ),
        TimerAction(
            period=26.0,
            actions=[
                LogInfo(msg="Launching audio sender; ensure host is receiving"),
                Node(
                    package='audio_sender',
                    executable='audio_sender_exe',
                    name='audio_sender_node',
                    output='screen',
                    parameters=[]
                )
            ]
        ),
        TimerAction(
            period=30.0,
            actions=[
                LogInfo(msg="Launching goals to follow"),
                Node(
                    package='follow_waypoints',
                    executable='follow_waypoints_exe',
                    name='follow_waypoints_node',
                    output='screen',
                    parameters=[]
                )
            ]
        )
    ])



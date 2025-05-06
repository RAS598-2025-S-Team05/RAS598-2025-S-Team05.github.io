from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model')
    map_yaml_file = LaunchConfiguration('map')

    turtlebot4_navigation_launch_dir = os.path.join(
        FindPackageShare('turtlebot4_navigation').find('turtlebot4_navigation'),
        'launch'
    )

    turtlebot4_viz_launch_dir = os.path.join(
        FindPackageShare('turtlebot4_viz').find('turtlebot4_viz'),
        'launch'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value=TextSubstitution(text='/rpi_07'),
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'model',
            default_value=TextSubstitution(text='lite'),
            description='Robot model'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=TextSubstitution(text='/home/vnolas82/ros2_ws/src/turtlebot4_mapping/maps/map_room.yaml'),
            description='Full path to map yaml file to load'
        ),

        # Launch Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot4_navigation_launch_dir, 'localization.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_file,
                'use_sim_time': TextSubstitution(text='false')
            }.items()
        ),

        # Launch RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot4_viz_launch_dir, 'view_robot.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'model': model
            }.items()
        ),
        
                # Launch Navigation (Nav2)
        # Launch Navigation (Nav2 stack)
	IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(
		os.path.join(turtlebot4_navigation_launch_dir, 'nav2.launch.py')
	    ),
	    launch_arguments={
		'namespace': namespace,
		'use_sim_time': TextSubstitution(text='false')
	    }.items()
	),
	
	Node(
            package='turtlebot4_mapping',
            executable='esp32_goal',
            name='esp32_goal',
            namespace=namespace,
            output='screen'
        )


    ])
    

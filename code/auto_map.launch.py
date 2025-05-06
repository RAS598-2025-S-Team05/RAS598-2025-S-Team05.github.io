from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model')

    # Directories
    turtlebot4_navigation_dir = FindPackageShare('turtlebot4_navigation').find('turtlebot4_navigation')
    turtlebot4_viz_dir = FindPackageShare('turtlebot4_viz').find('turtlebot4_viz')

    # Paths
    slam_launch = os.path.join(turtlebot4_navigation_dir, 'launch', 'slam.launch.py')
    viz_launch = os.path.join(turtlebot4_viz_dir, 'launch', 'view_robot.launch.py')

    # Map save path
    save_path = os.path.expanduser('~/ros2_ws/src/turtlebot4_mapping/maps/my_map')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=TextSubstitution(text='/rpi_07')),
        DeclareLaunchArgument('model', default_value=TextSubstitution(text='lite')),

        # ✅ Launch SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'namespace': namespace,
                'sync': TextSubstitution(text='false')
            }.items()
        ),

        # ✅ Launch RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch),
            launch_arguments={
                'namespace': namespace,
                'model': model
            }.items()
        ),

        # ✅ Teleop Keyboard
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '--remap', '/cmd_vel:=/rpi_07/cmd_vel'
            ],
            output='screen'
        ),

        # ✅ Obstacle Avoidance Node
        Node(
            package='my_project',
            executable='obstacle_avoid',
            name='obstacle_avoid',
            output='screen'
        ),

        # ✅ Map Saver Trigger Node (you'll create this)
        Node(
            package='turtlebot4_mapping',
            executable='auto_save',   # listens for keypress and saves map
            name='auto_save',
            output='screen',
            parameters=[{'save_path': save_path}]
        ),
    ])


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'my_robot.urdf.xacro')

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('goal_x', default_value='1.0', description='Goal position in x coordinate'),
        DeclareLaunchArgument('goal_y', default_value='1.0', description='Goal position in y coordinate'),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        ),

        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'world': os.path.join(
                    get_package_share_directory('my_robot_bringup'),
                    'worlds','Obstacle_Course',
                    'new_map_obstacle.world'
                )
            }.items(),
        ),

        # Spawn Entity Node in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot',
                       '-x','-8.0',
                       '-y', '1.0',
                       '-Y', '0.0',
                       '-z','0.01'],
        ),

        # Your Controller Node
        Node(
            package='my_robot_control',
            executable = 'planner',
            output = 'screen',
            parameters=[{
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
    }],
        ),

        Node(
            package='my_robot_control',
            executable='advanced_controller',
            output='screen',
            
        ),
    ])

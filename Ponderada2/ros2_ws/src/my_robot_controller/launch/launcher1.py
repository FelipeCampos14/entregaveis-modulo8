from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_gazebo   ',
            name='gazebo',
            output='screen'
        ),
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop',
            output='screen'
        ),
        Node(
            package='turtlebot3_cartographer',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
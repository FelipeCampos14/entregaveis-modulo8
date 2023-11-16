from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_navigation2',   
            executable='navigation2_node',
            parameters=[{'use_sim_time': True},{'map': mapa_salvo.yaml}]
            name='cartographer'
        ),
        Node(
            package='my_robot_controller',
            namespace='go_to_pose',
            executable='go_to_pose_node',
            name='go_to_pose',
        ),
    ])
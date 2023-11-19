from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        )
    ])
    
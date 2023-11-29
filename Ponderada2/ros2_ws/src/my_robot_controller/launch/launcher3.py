from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    set_pose = ExecuteProcess(
        cmd = ['ros2', 'run', 'my_robot_controller','set_pose_node'],
        shell=True
    )

    go_to_pose = ExecuteProcess(
        cmd = ['ros2', 'run', 'my_robot_controller','go_to_pose_node'],
        shell=True
    )

    return LaunchDescription([
        set_pose,
        go_to_pose,
    ])
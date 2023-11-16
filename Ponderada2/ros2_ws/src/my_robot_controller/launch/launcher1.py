from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("turtlebot3_gazebo"), '/launch', '/turtlebot3_world.launch.py']),
    )
    teleop = ExecuteProcess(
        cmd=['gnome-terminal','--','ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
        output='screen',
        shell=True
    )
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_cartographer"), '/launch', '/cartographer.launch.py']),
        launch_arguments={
            'use_sim_time':'True'
        }.items()
    )

    return LaunchDescription([
        gazebo,
        teleop,
        cartographer,   
    ])
    
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    mapa = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("turtlebot3_navigation2"), '/launch', '/navigation2.launch.py']),
                launch_arguments={
                    'use_sim_time':'True',
                    'map':'mapa3.yaml'
                }.items()
    )

    set_pose = Node(
        package='my_robot_controller',
        executable='set_pose_node'
    )

    go_to_pose = Node(
        package='my_robot_controller',
        executable='go_to_pose_node'
    )

    return LaunchDescription([
        mapa,
        set_pose,
        go_to_pose,

        RegisterEventHandler(
            OnProcessExit(
                target_action=set_pose,
                on_exit=[
                    go_to_pose,
                    LogInfo('Posição Setada')
                ]
            )
        )
    ])
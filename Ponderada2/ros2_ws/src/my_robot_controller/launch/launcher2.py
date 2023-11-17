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
                    'map':'../mapa/mapa_ponderada.yaml'
                }.items()
    )

    save_map = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'src/my_robot_controller/mapa/mapa_ponderada.yaml'],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        save_map,
        mapa   
    ])
    
    #ros2 run nav2_map_server map_saver_cli -f src/my_robot_controller/mapa/mapa_ponderada.yaml
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction)
from launch.substitutions import (FindExecutable)


def generate_launch_description():

    service = Node(
        package='swarm_intelligence_ros2',
        executable='server_test',
        output='screen',
    )

    client_cmd = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run ',
            'swarm_intelligence_ros2 ',
            'client_test ',
            '20 ',
            '0 ',
            '0 ',
        ]],
        shell=True
    )

    return LaunchDescription([
 
        service,

        TimerAction(
            period=3.0,
            actions=[client_cmd],
        )

    ])
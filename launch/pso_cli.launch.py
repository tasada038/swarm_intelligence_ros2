from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction)
from launch.substitutions import (FindExecutable)


def generate_launch_description():

    pso = Node(
        package='swarm_intelligence_ros2',
        executable='pso_service',
        output='screen',
    )

    client_cmd = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run ',
            'swarm_intelligence_ros2 ',
            'client_test ',
            '50 ',
            '0 ',
            '0 ',
        ]],
        shell=True
    )

    return LaunchDescription([
 
        pso,

        TimerAction(
            period=1.0,
            actions=[client_cmd],
        )

    ])
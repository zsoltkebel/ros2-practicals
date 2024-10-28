from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

namespace = 'turtlesim1'
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace=namespace,
            executable='turtlesim_node',
            name='sim'
        ),
        # Node(
        #     package='catch_the_turtle',
        #     namespace=namespace,
        #     executable='client',
        #     name='sim'
        # ),
        Node(
            package='catch_the_turtle',
            namespace=namespace,
            executable='pos',
            name='move'
        ),
        # Node(
        #     package='catch_the_turtle',
        #     namespace=namespace,
        #     executable='mover',
        #     name='move'
        # ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        ),
        # ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/turtlesim1/turtle1/cmd_vel
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--', 'ros2', 'run', 'turtlesim', 'turtle_teleop_key', '--ros-args', '--remap', '/turtle1/cmd_vel:=/turtlesim1/turtle1/cmd_vel'],
        #     output='screen'
        # )
    ])
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_nodes',
            executable='user_input',
            name='user_input',
            output='screen',
            prefix='gnome-terminal --title="User Input" -- '
        ),
        Node(
            package='query_generation',
            executable='query_generation',
            name='query_generation',
            output='screen',
            prefix='gnome-terminal --title="Query Generation Listener" -- '
        ),
        Node(
            package='intent_recognition',
            executable='intent_recognition',
            name='intent_recognition',
            output='screen',
            prefix='gnome-terminal --title="Intent Recognition" -- '
        ),
                Node(
            package='inner_speech',
            executable='inner_speech',
            name='inner_speech',
            output='screen',
            prefix='gnome-terminal --title="Inner Speech" -- '
        ),
    ])

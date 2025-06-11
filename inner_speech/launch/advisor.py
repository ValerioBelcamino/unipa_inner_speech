import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    input_mode_arg = DeclareLaunchArgument(
        'use_audio',
        default_value='false',
        description='Set to "true" to use speech input instead of text input.'
    )

    use_audio = LaunchConfiguration('use_audio')

    return LaunchDescription([
        input_mode_arg,

        Node(
            package='perception_nodes',
            executable='user_input',
            name='user_input',
            output='screen',
            prefix='gnome-terminal --title="User Input (Text)" -- ',
            condition=UnlessCondition(use_audio)
        ),

        Node(
            package='perception_nodes',
            executable='speech_recognition_node',
            name='speech_input',
            output='screen',
            prefix='gnome-terminal --title="Speech Input" -- ',
            condition=IfCondition(use_audio)
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
        Node(
            package='explainability',
            executable='explainability',
            name='explainability',
            output='screen',
            prefix='gnome-terminal --title="Explainability" -- '
        ),
        Node(
            package='scope_detection',
            executable='scope_detection',
            name='scope_detection',
            output='screen',
            prefix='gnome-terminal --title="Scope Detection" -- '
        ),
    ])

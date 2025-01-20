import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inner_speech',
            executable='user_input',
            name='user_input',
            output='screen',
            prefix='gnome-terminal --title="User Input" -- '
        ),
        Node(
            package='neo4j_nodes',
            executable='listener_dish_info',
            name='listener_dish_info',
            output='screen',
            prefix='gnome-terminal --title="Dish Info Listener" -- '
        ),
        Node(
            package='neo4j_nodes',
            executable='listener_user_insertion',
            name='listener_user_insertion',
            output='screen',
            prefix='gnome-terminal --title="User Insertion Listener" -- '
        ),
        Node(
            package='neo4j_nodes',
            executable='listener_meal_prep',
            name='listener_meal_prep',
            output='screen',
            prefix='gnome-terminal --title="Meal Prep Listener" -- '
        ),
        Node(
            package='clingo_nodes',
            executable='clingo_listener',
            name='clingo_listener',
            output='screen',
            prefix='gnome-terminal --title="Clingo Listener" -- '
        ),
        Node(
            package='explainability',
            executable='explainability_listener',
            name='explainability_listener',
            output='screen',
            prefix='gnome-terminal --title="Explainability Listener" -- '
        ),
    ])

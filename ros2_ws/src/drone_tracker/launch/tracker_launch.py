from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():    
    return LaunchDescription([
        DeclareLaunchArgument(
            'tracker_mode',
            default_value='kcf',
            choices=['sort', 'deepsort', 'mosse', 'csrt', 'kcf', 'na'],
            description='Tracker Mode'
        ),

        Node(
            package='drone_tracker',
            namespace='cam',
            executable='telescope',
            name='run',
        ),

        Node(
            package='drone_tracker',
            namespace='detect',
            executable='detect',
            name='run',
            parameters=[{'tracker_mode':LaunchConfiguration('tracker_mode')}],
        ),

        Node(
            package='drone_tracker',
            namespace='track',
            executable='track_cv',
            name='run',
            condition=LaunchConfigurationEquals('tracker_mode', 'mosse'),
            parameters=[{'tracker_mode':LaunchConfiguration('tracker_mode')}] 
        ),
        Node(
            package='drone_tracker',
            namespace='track',
            executable='track_cv',
            name='run',
            condition=LaunchConfigurationEquals('tracker_mode', 'kcf'),
            parameters=[{'tracker_mode':LaunchConfiguration('tracker_mode')}] 
        ),
        Node(
            package='drone_tracker',
            namespace='track',
            executable='track_cv',
            name='run',
            condition=LaunchConfigurationEquals('tracker_mode', 'csrt'),
            parameters=[{'tracker_mode':LaunchConfiguration('tracker_mode')}] 
        ),



        Node(
            package='drone_tracker',
            namespace='track',
            executable='track_sort',
            name='run',
            condition=LaunchConfigurationEquals('tracker_mode', 'sort')
        ),


        Node(
            package='drone_tracker',
            namespace='track',
            executable='track_dsort',
            name='run',
            condition=LaunchConfigurationEquals('tracker_mode', 'deepsort')
        ),
        Node(
            package='drone_tracker',
            namespace='view',
            executable='viewer',
            name='run'
        )
    ])
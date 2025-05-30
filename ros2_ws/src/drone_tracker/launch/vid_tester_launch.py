from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():    
    return LaunchDescription([

        DeclareLaunchArgument(
            'tracker_mode',
            default_value='mosse',
            choices=['sort', 'deepsort', 'mosse', 'csrt', 'kcf', 'na'],
            description='Tracker Mode'
        ),

        DeclareLaunchArgument(
            'video_index',
            default_value='0',
            description='Choose which video from index 0 to 4'
        ),

        Node(
            package='drone_tracker',
            namespace='detect',
            executable='detect',
            name='run',
            parameters=[{'tracker_mode':LaunchConfiguration('tracker_mode')}] 
        ),

        TimerAction(
            period=10.0,  # 30 seconds delay
            actions=[

                # Node for video playback
                Node(
                    package='drone_tracker',
                    namespace='vid_playback',
                    executable='vid_playback',
                    name='run',
                    parameters=[{'video_index': LaunchConfiguration('video_index')}]
                )]),
        


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
        ),
        Node(
            package='drone_tracker',
            namespace='data',
            executable='data',
            name='run',
            parameters=[{'video_index': LaunchConfiguration('video_index')},{'tracker_mode':LaunchConfiguration('tracker_mode')}]
        ),
    ])
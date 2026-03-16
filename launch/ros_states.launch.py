"""Launch file for ROS2 State Observer."""

import os
import webbrowser
import threading

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _open_browser(context, *args, **kwargs):
    port = LaunchConfiguration('port').perform(context)
    open_browser = LaunchConfiguration('open_browser').perform(context)
    if open_browser.lower() in ('true', '1', 'yes'):
        url = f'http://localhost:{port}'

        def _delayed_open():
            import time
            time.sleep(2)
            webbrowser.open(url)

        threading.Thread(target=_delayed_open, daemon=True).start()
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='5050',
            description='Web server port number',
        ),
        DeclareLaunchArgument(
            'update_interval',
            default_value='5000',
            description='Dashboard update interval in milliseconds',
        ),
        DeclareLaunchArgument(
            'open_browser',
            default_value='true',
            description='Open web browser on launch',
        ),
        DeclareLaunchArgument(
            'domain_id',
            default_value='0',
            description='Initial ROS_DOMAIN_ID (0-232)',
        ),
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'ros_states.app',
                '--port', LaunchConfiguration('port'),
                '--update-interval', LaunchConfiguration('update_interval'),
                '--domain-id', LaunchConfiguration('domain_id'),
            ],
            output='screen',
        ),
        OpaqueFunction(function=_open_browser),
    ])

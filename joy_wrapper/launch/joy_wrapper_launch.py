import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joy_wrapper_params = os.path.join(
        get_package_share_directory('joy_wrapper'),
        'config',
        'joy.yaml'
    )
    
    joy_node = Node(
        namespace='/joy',
        package='joy',
        executable='joy_node',
        name='joy_node'#,
        #arguments=['--ros-args', '--log-level', ["debug"]]#,
        #prefix=['gdb -ex run --args']
        #prefix=['xterm -e gdb -ex run --args']
        #prefix=['gnome-terminal -x gdb -ex run --args']
    )

    joy_wrapper_node = Node(
        namespace='/joy_wrapper',
        package='joy_wrapper',
        executable='joy_wrapper_node',
        name='joy_wrapper_node',
        parameters=[joy_wrapper_params],
        arguments=['--ros-args', '--log-level', ["debug"]]#,
        #prefix=['gdb -ex run --args']
        #prefix=['xterm -e gdb -ex run --args']
        #prefix=['gnome-terminal -x gdb -ex run --args']
    )

    ld.add_action(joy_node)
    ld.add_action(joy_wrapper_node)

    return ld
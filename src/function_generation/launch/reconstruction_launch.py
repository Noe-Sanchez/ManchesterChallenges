import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('function_generation'),
                                    'config',
                                    'params.yaml')

    gen_node = Node(
        package='function_generation',
        executable='generator',
        name = 'signal_generator',
        output='screen',
        parameters=[config]
    )

    rec_node = Node(
        package='function_generation',
        executable='reconstructor',
        name = 'signal_reconstructor',
        output='screen',
        parameters=[config]
    )

    l_d = LaunchDescription([gen_node, rec_node])
    return l_d

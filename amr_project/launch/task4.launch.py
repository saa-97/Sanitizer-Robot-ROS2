from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
   # Launch file to launch the sanitize node
    my_node1 = Node(
            package='amr_project',
            executable='sanitize',
            name='sanitize',
            output='screen',
            emulate_tty=True,
        )
          
    # Creation of the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(my_node1)

    return ld
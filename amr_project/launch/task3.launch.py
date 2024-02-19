from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
   # Launch file to launch the laserscan, localization and route_manager nodes simultaneously

    my_node1 = Node(
            package='amr_project',
            executable='localize',
            name='localize',
        )
    my_node2 = Node(
            package='amr_project',
            executable='laserscan',
            name='rdv',
        )
    my_node3 = Node(
            package='amr_project',
            executable='route_node',
            name='route_manager',
        )
   
   
   
    # Creation of the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(my_node2)
    ld.add_action(my_node3)
    ld.add_action(my_node1)

    return ld
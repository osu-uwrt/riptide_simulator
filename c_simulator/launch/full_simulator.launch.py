import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
import os

physics_launch = os.path.join(get_package_share_directory('c_simulator'), 'launch', 'physics_simulator.launch.py')
camera_launch = os.path.join(get_package_share_directory('camera_faker'), 'launch', 'zedfaker.launch.py')
rviz_launch = os.path.join(get_package_share_directory('riptide_rviz'), 'launch', 'rviz_start.launch.py')

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="talos",
            description="Name of the vehicle",
        ),
        
        #physics simulator
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(physics_launch),
            launch_arguments=[
                ('robot', LC('robot'))
            ]
        ),
        
        #camera faker
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(camera_launch),
            launch_arguments=[
                ('robot', LC('robot'))
            ]
        ),
        
        #rviz
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rviz_launch),
            launch_arguments=[
                ('robot', LC('robot'))
            ]
        )
    ])

# ros2 launch ./sim.launch.py      (needs ros_gz_sim + ros_gz_bridge for Gazebo Harmonic)
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

here = os.path.dirname(os.path.abspath(__file__))

def generate_launch_description():
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {os.path.join(here, "world.sdf")}'}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[f'/wd/rotor_{i}/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double' for i in range(1, 5)]
                  + ['/world/wd_world/model/wd/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'])
    return LaunchDescription([SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', here), gz, bridge])

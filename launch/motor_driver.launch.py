import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
          'params_file',
          default_value=os.path.join(get_package_share_directory('motor_driver'), 'config', 'params.yaml'),
          description='Full path to the ROS2 parameters file to use for all launched nodes')
    param_substitutions = {
          'use_sim_time': use_sim_time,
          }
    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    ld = LaunchDescription()
    node = Node(
      package='motor_driver',
      executable='motor_driver',
      output='screen',
      parameters=[configured_params],
      # arguments=['--ros-args', '--log-level', 'debug']
      # prefix=['xterm -e gdb -ex run --args']
    )
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(node)
    return ld

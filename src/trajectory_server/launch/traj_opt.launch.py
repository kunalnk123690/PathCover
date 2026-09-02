import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Receding-horizon GCOPTER trajectory planner. Drop-in replacement for
    # trajectory_server_node: same topics, same
    # TrajectoryCommand/PolytopeArray/Odometry message types.
    traj_opt_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='traj_opt_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('trajectory_server'),
                    'config',
                    'gcopter_params.yaml')]
    )

    return LaunchDescription([
        traj_opt_node,
    ])

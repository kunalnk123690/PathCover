from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os



def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_dir = os.path.join(get_package_share_directory('quadrotor_description'), 'config', 'quadrotor_visual.rviz')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(
                    'quadrotor_description'),
                    'urdf',
                    'quadrotor.xacro.urdf']
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    default_world = os.path.join(
        get_package_share_directory('quadrotor_description'),
        'worlds',
        'empty.world'
        # 'warehouse.world'
    )    
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )    

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )


    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-v 4 -r --headless-rendering ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'quadrotor',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.1',
                                   '-R', '0.0',
                                   '-P', '0.0',
                                   '-Y', '0.0',
                                ],
                        output='screen')
    

    node_ground_truth = Node(
        package='quadrotor_description',
        executable='ground_truth_node',
        output='screen'
    )


    # Convert ignition/gz msgs to ros msgs:
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/quadrotor/ground_truth@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/quadrotor/IMU@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/quadrotor/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/quadrotor/velodyne/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/quadrotor/realsense/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/quadrotor/realsense/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/quadrotor/realsense/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/quadrotor/realsense/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/quadrotor/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/quadrotor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )


    teleop_node = Node(
        package='quadrotor_teleop',
        executable='quadrotor_teleop_node',
        output='screen'
    )


    node_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')


    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        sensor_bridge,
        node_robot_state_publisher,
        node_ground_truth,
        node_rviz,
        teleop_node,
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_kdl_package')
    params_file = os.path.join(pkg_share, 'config', 'homework2_params.yaml')

    # Punto 1a/1b (commenti in italiano):
    # - Questo launch avvia il nodo ros2_kdl_node caricando i parametri dal file YAML.
    # - Espone alcuni argomenti per sovrascrivere i parametri più usati a runtime:
    #   cmd_interface (position|velocity|effort), ctrl (velocity_ctrl|velocity_ctrl_null), lambda.

    # Declare launch arguments
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface',
        default_value='position',
        description='Command interface: position, velocity, or effort'
    )
    
    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='velocity_ctrl',
        description='Velocity controller type: velocity_ctrl, velocity_ctrl_null or vision'
    )
    
    lambda_arg = DeclareLaunchArgument(
        'lambda',
        default_value='1.0',
        description='Scaling factor for null-space optimization'
    )

    aruco_topic_arg = DeclareLaunchArgument(
        'aruco_topic',
        default_value='/aruco_single/pose',
        description='Topic name publishing the ArUco pose (geometry_msgs/PoseStamped)'
    )

    vision_velocity_limit_arg = DeclareLaunchArgument(
        'vision_velocity_limit',
        default_value='1.0',
        description='Absolute joint velocity limit for the vision controller (rad/s). Use 0 to disable.'
    )

    # Add missing trajectory parameters
    traj_duration_arg = DeclareLaunchArgument(
        'traj_duration',
        default_value='1.5',
        description='Trajectory duration in seconds'
    )
    
    acc_duration_arg = DeclareLaunchArgument(
        'acc_duration',
        default_value='0.5',
        description='Acceleration phase duration in seconds'
    )
    
    total_time_arg = DeclareLaunchArgument(
        'total_time',
        default_value='1.5',
        description='Total execution time in seconds'
    )
    
    trajectory_len_arg = DeclareLaunchArgument(
        'trajectory_len',
        default_value='150',
        description='Number of trajectory points'
    )
    
    Kp_arg = DeclareLaunchArgument(
        'Kp',
        default_value='5.0',
        description='Proportional gain for position control'
    )

    node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        output='screen',
        parameters=[
            params_file,
            {
                'cmd_interface': LaunchConfiguration('cmd_interface'),
                'ctrl': LaunchConfiguration('ctrl'),
                'lambda': LaunchConfiguration('lambda'),
                'aruco_topic': LaunchConfiguration('aruco_topic'),
                'vision_velocity_limit': LaunchConfiguration('vision_velocity_limit'),
                'traj_duration': LaunchConfiguration('traj_duration'),
                'acc_duration': LaunchConfiguration('acc_duration'),
                'total_time': LaunchConfiguration('total_time'),
                'trajectory_len': LaunchConfiguration('trajectory_len'),
                'Kp': LaunchConfiguration('Kp'),
            }
        ]
    )
    # ============================================
    # HOMEWORK 2c: Service bridge to set ArUco pose
    # ============================================
    # We expose the Ignition Gazebo service `/world/iiwa_aruco_world/set_pose`
    # through ROS 2 using ros_gz_bridge so that a user can call:
    #   ros2 service call /world/iiwa_aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'aruco_tag_front'}, pose: {position: {x: 0.1, y: -0.5, z: 0.6}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    # to move the marker. Entity name must match <name> in the world file.
    # NOTE: The parameter_bridge argument syntax for services is:
    #   /service_name@ros_service_type@gz_service_request_type@gz_service_response_type
    # For SetEntityPose the Gazebo request type is ignition.msgs.Pose and response is ignition.msgs.Boolean.

    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='aruco_set_pose_bridge',
        output='screen',
        arguments=[
            # Simplified syntax - let the bridge auto-detect Gazebo message types
            '/world/iiwa_aruco_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ]
    )

    return LaunchDescription([
        cmd_interface_arg,
        ctrl_arg,
        lambda_arg,
        aruco_topic_arg,
        vision_velocity_limit_arg,
        traj_duration_arg,
        acc_duration_arg,
        total_time_arg,
        trajectory_len_arg,
        Kp_arg,
        node,
        set_pose_bridge
    ])

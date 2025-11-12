from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    ============================================
    HOMEWORK 2b: Bridge Camera Gazebo -> ROS 2
    ============================================
    
    Questo nodo fa il bridge dei topic della camera dal sistema Gazebo Transport
    ai topic ROS 2 standard.
    
    NOTA: Gazebo pubblica con il nome corto del topic (definito nel tag <topic>)
    invece del path completo world/model/link/sensor.
    
    GAZEBO TRANSPORT (input):
      /wrist_camera
      /wrist_camera/camera_info
    
    ROS 2 (output):
      /wrist_camera/image_raw
      /wrist_camera/camera_info
    """
    
    # Topic Gazebo (nomi corti)
    gz_image = '/wrist_camera'
    gz_info = '/wrist_camera/camera_info'
    
    # Topic ROS 2 (nomi standard)
    ros_image = '/wrist_camera/image_raw'
    ros_info = '/wrist_camera/camera_info'
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrist_camera_bridge',
        output='screen',
        arguments=[
            # Sintassi: topic_gz@tipo_ros[tipo_gz
            # Il [ significa unidirezionale: Gazebo -> ROS 2
            f'{gz_image}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            # Rinomina solo l'immagine (camera_info ha già il nome giusto)
            (gz_image, ros_image),
        ],
    )
    
    return LaunchDescription([bridge])

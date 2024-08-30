from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import datetime

home_dir = os.environ["HOME"]
txt_dir = home_dir + '/Voc/ORBvoc.bin'
yaml_dir = home_dir + '/Voc/config/RealSense_T265.yaml'
equalize = 'true'

def generate_launch_description():
    left_topic_arg = DeclareLaunchArgument('left_topic', default_value='/camera/camera/infra1/image_rect_raw')
    right_topic_arg = DeclareLaunchArgument('right_topic', default_value='/camera/camera/infra2/image_rect_raw')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/camera/camera/imu')     

    orb3 = Node(
                package='gr1-localization-orb-slam3',
                executable='stereo_inertial',
                name='orb_slam3',
                output='screen',
                parameters=[{'left_topic' : LaunchConfiguration('left_topic')},
                            {'right_topic' : LaunchConfiguration('right_topic')},
                            {'imu_topic' : LaunchConfiguration('imu_topic')}],
                arguments=[txt_dir, yaml_dir, equalize]
                )

    # Publish static tf from map to odom
    # Map -> odom -> base_link
    arg_base = ['0', '0', '0', '0', '0', '0', 'map', 'odom']    
    tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='my_static_tf_publisher',
            output='screen',  # For debugging purposes
            arguments= arg_base
        )

    return LaunchDescription([left_topic_arg, right_topic_arg, imu_topic_arg, tf, orb3])

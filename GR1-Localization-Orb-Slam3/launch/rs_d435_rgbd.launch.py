from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import datetime

home_dir = os.environ["HOME"]
txt_dir = home_dir + '/Voc/ORBvoc.bin'
yaml_dir = home_dir + '/Voc/config/RealSense_D435i_RGBD.yaml'

def generate_launch_description():
    rgb_topic_arg = DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/camera/camera/depth/image_rect_raw')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/camera/camera/imu')     

    orb3 = Node(
                package='gr1-localization-orb-slam3',
                executable='rgbd-inertial',
                name='orb_slam3',
                output='screen',
                parameters=[{'rgb_topic' : LaunchConfiguration('rgb_topic')},
                            {'depth_topic' : LaunchConfiguration('depth_topic')},
                            {'imu_topic' : LaunchConfiguration('imu_topic')}],
                arguments=[txt_dir, yaml_dir, 'true']
                )

    return LaunchDescription([rgb_topic_arg, depth_topic_arg, imu_topic_arg, orb3])

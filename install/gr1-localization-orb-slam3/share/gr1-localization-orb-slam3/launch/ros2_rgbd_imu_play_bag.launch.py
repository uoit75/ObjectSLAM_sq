from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import datetime

home_dir = os.environ["HOME"]
bin_dir = home_dir + '/Voc/ORBvoc.bin'
yaml_dir = home_dir + '/Voc/config/RealSense_D435i_RGBD.yaml'
arg_base = ['0', '0', '0', '0', '0', '0', 'map', 'odom']

def generate_launch_description():    
    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/0415_2.bag"
    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/indoor_0619_unaligned.bag"
    cmd_string = "ros2 bag play " + home_dir + "/ros2bag/indoor_0624_2.bag"
    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/outdoor_0624_2.bag"
    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/0411_1.bag"
    cmd  = ExecuteProcess(
        cmd = [cmd_string],
        output="screen",
        shell=True
    )
    rgb_topic_arg = DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/camera/camera/imu')  

    orb3 = Node(
                package='gr1-localization-orb-slam3',
                executable='rgbd-inertial',
                name='orb_slam3',
                output='screen',
                parameters=[{'rgb_topic' : LaunchConfiguration('rgb_topic')},
                            {'depth_topic' : LaunchConfiguration('depth_topic')},
                            {'imu_topic' : LaunchConfiguration('imu_topic')}],
                arguments=[bin_dir, yaml_dir, 'true']
                )
    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='my_static_tf_publisher',
        output='screen',  # For debugging purposes
        arguments= arg_base
    )

    return LaunchDescription([tf, rgb_topic_arg, depth_topic_arg, imu_topic_arg, orb3, cmd])

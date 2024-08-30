from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import datetime

home_dir = os.environ["HOME"]

def generate_launch_description():
    #for 1124
    # rgb_topic_arg = DeclareLaunchArgument('rgb_topic', default_value='/feynman_camera/M1CF115T22311037/rgb/image_rect_color')
    # depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/feynman_camera/M1CF115T22311037/depth/image_raw')
    # imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/feynman_camera/M1CF115T22311037/imu_single') 
    #for 1130
    rgb_topic_arg = DeclareLaunchArgument('rgb_topic', default_value='/feynman_camera/M1CF115T22311037/rgb/image_rect_color')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/feynman_camera/M1CF115T22311037/depth/image_raw')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/feynman_camera/M1CF115T22311037/imu_single')     
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/odometry')
    rtk_topic_arg = DeclareLaunchArgument('rtk_topic', default_value='/gps_data')

    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/1121_3.bag" 
    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/1124_4.bag"
    cmd_string = "ros2 bag play " + home_dir + "/Work/bag/1124_4.bag"
    # cmd_string = "ros2 bag play " + home_dir + "/ros2bag/1214_2.bag"
    cmd  = ExecuteProcess(
        cmd = [cmd_string],
        output="both",
        shell=True
    )

    orb3 = Node(
                package='ros2_rgbd_imu_rtk_odom',
                executable='ros2_rgbd_imu_rtk_odom_node',
                name='orb_fusion',
                output='screen',
                parameters=[{'rgb_topic' : LaunchConfiguration('rgb_topic')},
                            {'depth_topic' : LaunchConfiguration('depth_topic')},
                            {'imu_topic' : LaunchConfiguration('imu_topic')},
                            {'odom_topic' : LaunchConfiguration('odom_topic')},
                            {'rtk_topic' : LaunchConfiguration('rtk_topic')}],
                arguments=['./src/avengers/location/slam/orb_fusion/orb-slam3/Vocabulary/ORBvoc.bin', './src/avengers/location/slam/orb_fusion/ros2_rgbd_imu_rtk_odom/config/feynman_old.yaml', 'true']
                )

    return LaunchDescription([cmd, rgb_topic_arg, depth_topic_arg, imu_topic_arg, odom_topic_arg, rtk_topic_arg, orb3])
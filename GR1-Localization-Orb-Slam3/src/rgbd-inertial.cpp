#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rgbd-inertial-node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    std::string temp(argv[3]);
    if(temp == "false") visualization = false;
    #ifdef RgbdInertial
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, visualization);
    #endif
    // ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);
    #ifdef Rgbd
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);
    #endif


    auto node = std::make_shared<RgbdInertialNode>(&pSLAM);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

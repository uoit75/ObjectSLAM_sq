#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include "rgbd_yolo_node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    std::string temp(argv[3]);
    if(temp == "false") visualization = false;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    auto node = std::make_shared<RgbdYoloNode>(&pSLAM);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

#include "rgbd_yolo_node.hpp"
#include <chrono>

#include <opencv2/core/core.hpp>

#include "yolov8_ros/msg/detection_result_msg.hpp"


using namespace std::placeholders;

RgbdYoloNode::RgbdYoloNode(ORB_SLAM3::System *pSLAM) :
    Node("ORB_SLAM3_ROS2"),
    pSLAM_(pSLAM)
    // window_size_(5)
{

    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "camera/Rgb");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "camera/Depth");

    subDetectionResult_ = this->create_subscription<yolov8_ros::msg::DetectionResultMsg>(
    "detection_result", 10, std::bind(&RgbdYoloNode::detectionResultCallback, this, _1));


    cout<<"rgb_topic_: "<<rgb_topic_<<std::endl;
    cout<<"depth_topic_: "<<depth_topic_<<std::endl;
    subImgRgb_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, rgb_topic_);
    subImgDepth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, depth_topic_);
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *subImgRgb_, *subImgDepth_);
    syncApproximate->registerCallback(&RgbdYoloNode::GrabRGBD, this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    clear_imu_point_flag_ = true;
    std::cout << "RGBD-Yolo node has been initialized!" << std::endl;
    Identity = Sophus::SE3f(R_, t_);
    pose_.header.frame_id = "map";
    path_.header.frame_id = "map";
    trans_.header.frame_id = "odom";
    trans_.child_frame_id = "base_link";

}

RgbdYoloNode::~RgbdYoloNode()
{
    // Stop all threads
    pSLAM_->Shutdown();

}




// void RgbdYoloNode::detectionResultCallback(const yolov8_ros::msg::DetectionResultMsg::SharedPtr msg)
// {
    
//     int image_counter = 0; // 用于生成唯一的文件名

//     for (const auto& object : msg->objects) {
//         int label = object.label;
//         float confidence = object.confidence;
//         cv::Rect bbox;

//         // 从四个角点转换为 cv::Rect
//         if (object.bbox_points.size() == 4) {
//             int x_min = std::min({object.bbox_points[0].x, object.bbox_points[1].x, object.bbox_points[2].x, object.bbox_points[3].x});
//             int y_min = std::min({object.bbox_points[0].y, object.bbox_points[1].y, object.bbox_points[2].y, object.bbox_points[3].y});
//             int x_max = std::max({object.bbox_points[0].x, object.bbox_points[1].x, object.bbox_points[2].x, object.bbox_points[3].x});
//             int y_max = std::max({object.bbox_points[0].y, object.bbox_points[1].y, object.bbox_points[2].y, object.bbox_points[3].y});

//             // 创建 cv::Rect 对象
//             bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

//             // 打印 BBox 信息
//             std::cout << "BBox: x=" << bbox.x << ", y=" << bbox.y
//                       << ", width=" << bbox.width << ", height=" << bbox.height << std::endl;
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Unexpected number of bbox points: %ld", object.bbox_points.size());
//             continue;
//         }

//         // // 输出图像的步幅、宽度、高度等信息进行调试
//         // std::cout << "Image width: " << object.masks_instance.width << std::endl;
//         // std::cout << "Image height: " << object.masks_instance.height << std::endl;
//         // std::cout << "Image step: " << object.masks_instance.step << std::endl;
//         // std::cout << "Expected step: " << object.masks_instance.width << std::endl; // 对于灰度图，步幅应等于宽度

//         // 将 ros message 中的 mono8 mask 数据转换为 OpenCV 的 Mat 对象
//         cv_bridge::CvImagePtr cv_ptr;
//         try {
//             cv_ptr = cv_bridge::toCvCopy(object.masks_instance, sensor_msgs::image_encodings::MONO8);
//         } catch (cv_bridge::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//             continue;
//         }

//         cv::Mat masks_instance = cv_ptr->image;

//         // 二值化图像，以确保只有黑白像素
//         cv::Mat binary_mask;
//         cv::threshold(masks_instance, binary_mask, 1, 255, cv::THRESH_BINARY);

//         // 查找轮廓，RETR_EXTERNAL表示只检测最外层的轮廓，CHAIN_APPROX_SIMPLE表示将轮廓压缩到最重要的点
//         std::vector<std::vector<cv::Point>> contours;
//         cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         // 创建一个空白图像，用于绘制 maskpoints
//         cv::Mat output_image = cv::Mat::zeros(masks_instance.size(), CV_8UC3);

//         // 将每个轮廓绘制到 output_image 上
//         for (const auto& contour : contours) {
//             cv::drawContours(output_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
//         }

//         // 生成唯一的文件名并保存图片
//         std::string file_name = "output_mask_" + std::to_string(image_counter++) + ".png";
//         cv::imwrite(file_name, output_image);

//         // 将绘制好的 maskpoints 存入 DetectedObject
//         std::vector<std::vector<cv::Point>> maskpoints = contours;
//         DetectedObject detected_object(label, confidence, bbox, maskpoints);
//         detected_objects.push_back(detected_object);

//         // std::cout << "start print" << std::endl;
//         // detected_object.print();  // 打印检测到的对象的信息
//     }

//     // 现在你可以使用 detected_objects 进行进一步处理
// }

void RgbdYoloNode::detectionResultCallback(const yolov8_ros::msg::DetectionResultMsg::SharedPtr msg)
{
    
    int image_counter = 0; // 用于生成唯一的文件名

    for (const auto& object : msg->objects) {
        int label = object.label;
        float confidence = object.confidence;
        cv::Rect bbox;

        // 从四个角点转换为 cv::Rect
        if (object.bbox_points.size() == 4) {
            int x_min = std::min({object.bbox_points[0].x, object.bbox_points[1].x, object.bbox_points[2].x, object.bbox_points[3].x});
            int y_min = std::min({object.bbox_points[0].y, object.bbox_points[1].y, object.bbox_points[2].y, object.bbox_points[3].y});
            int x_max = std::max({object.bbox_points[0].x, object.bbox_points[1].x, object.bbox_points[2].x, object.bbox_points[3].x});
            int y_max = std::max({object.bbox_points[0].y, object.bbox_points[1].y, object.bbox_points[2].y, object.bbox_points[3].y});

            // 创建 cv::Rect 对象
            bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

            // 打印 BBox 信息
            std::cout << "BBox: x=" << bbox.x << ", y=" << bbox.y
                      << ", width=" << bbox.width << ", height=" << bbox.height << std::endl;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unexpected number of bbox points: %ld", object.bbox_points.size());
            continue;
        }

        // // 输出图像的步幅、宽度、高度等信息进行调试
        // std::cout << "Image width: " << object.masks_instance.width << std::endl;
        // std::cout << "Image height: " << object.masks_instance.height << std::endl;
        // std::cout << "Image step: " << object.masks_instance.step << std::endl;
        // std::cout << "Expected step: " << object.masks_instance.width << std::endl; // 对于灰度图，步幅应等于宽度

        // 将 ros message 中的 mono8 mask 数据转换为 OpenCV 的 Mat 对象
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(object.masks_instance, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            continue;
        }

        cv::Mat masks_instance = cv_ptr->image;

        // 二值化图像，以确保只有黑白像素
        cv::Mat binary_mask;
        cv::threshold(masks_instance, binary_mask, 1, 255, cv::THRESH_BINARY);

        // 查找轮廓，RETR_EXTERNAL表示只检测最外层的轮廓，CHAIN_APPROX_SIMPLE表示将轮廓压缩到最重要的点
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 创建一个空白图像，用于绘制 maskpoints
        cv::Mat output_image = cv::Mat::zeros(masks_instance.size(), CV_8UC3);

        // 将每个轮廓绘制到 output_image 上
        for (const auto& contour : contours) {
            cv::drawContours(output_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
        }

        // 生成唯一的文件名并保存图片
        std::string file_name = "output_mask_" + std::to_string(image_counter++) + ".png";
        cv::imwrite(file_name, output_image);

        // // 将绘制好的 maskpoints 存入 DetectedObject
        // std::vector<std::vector<cv::Point>> maskpoints = contours;
        // DetectedObject detected_object(label, confidence, bbox, maskpoints);
        // detected_objects.push_back(detected_object);

        // std::cout << "start print" << std::endl;
        // detected_object.print();  // 打印检测到的对象的信息
    }

    // 现在你可以使用 detected_objects 进行进一步处理
}








void RgbdYoloNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD){
    // std::cout << "GrabRGBD function called" << std::endl;
    static double tlast = 0.0;
    double tImrgb = rclcpp::Time(msgRGB->header.stamp).seconds();
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        #ifdef WEBOTS
        cv_ptrRGB = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGRA8);
        cv_ptrD = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
        #endif
        #ifdef REALROBOTS
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        cv_ptrD = cv_bridge::toCvShare(msgD);
        #endif
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imClahe, imGray;
    try{
        #ifdef WEBOTS
        cv::cvtColor(cv_ptrRGB->image, imGray, cv::COLOR_BGRA2GRAY);
        clahe_->apply(imGray, imClahe);
        #endif
        #ifdef REALROBOTS
        cv::cvtColor(cv_ptrRGB->image, imGray, cv::COLOR_RGB2GRAY);
        clahe_->apply(imGray, imClahe);
        #endif
    }
    catch(cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = pSLAM_->TrackRGBD(imClahe, cv_ptrD->image, tImrgb);
    double dt = tImrgb - tlast;
    tlast = tImrgb;
    pose_.header.stamp = msgRGB->header.stamp;
    path_.header.stamp = msgRGB->header.stamp;
    trans_.header.stamp = msgRGB->header.stamp;

}


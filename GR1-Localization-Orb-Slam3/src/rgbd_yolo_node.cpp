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
    "detection_result", 1, std::bind(&RgbdYoloNode::detectionResultCallback, this, _1));


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



void RgbdYoloNode::detectionResultCallback(const yolov8_ros::msg::DetectionResultMsg::SharedPtr msg)
{


    for (const auto& object : msg->objects) {
        int label = object.label;
        float confidence = object.confidence;
        cv::Rect bbox;

        // 从四个角点转换为 cv::Rect
        // if (object.bbox_points.size() == 4) {
        //     int x_min = std::min({object.bbox_points[0].x, object.bbox_points[1].x, object.bbox_points[2].x, object.bbox_points[3].x});
        //     int y_min = std::min({object.bbox_points[0].y, object.bbox_points[1].y, object.bbox_points[2].y, object.bbox_points[3].y});
        //     int x_max = std::max({object.bbox_points[0].x, object.bbox_points[1].x, object.bbox_points[2].x, object.bbox_points[3].x});
        //     int y_max = std::max({object.bbox_points[0].y, object.bbox_points[1].y, object.bbox_points[2].y, object.bbox_points[3].y});

            // // 创建 cv::Rect 对象
            // bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);


        if (object.bbox_points.size() != 4) 
        {
            RCLCPP_WARN(rclcpp::get_logger("yolov8_ros"), "Incorrect number of points for bbox: %ld. Expected 4.", object.bbox_points.size());
            continue;  // 跳过此对象，继续处理下一个对象
        }
        // 假设四个点是按顺序存储的：左上、右上、右下、左下
        const geometry_msgs::msg::Point& p1 = object.bbox_points[0];  // 左上角
        const geometry_msgs::msg::Point& p2 = object.bbox_points[1];  // 右上角
        const geometry_msgs::msg::Point& p4 = object.bbox_points[3];  // 左下角

        // 计算宽度和高度
        int x = static_cast<int>(p1.x);
        int y = static_cast<int>(p1.y);
        int width = static_cast<int>(p2.x - p1.x);  // 右上角的x - 左上角的x
        int height = static_cast<int>(p4.y - p1.y);  // 左下角的y - 左上角的y
        

        // 创建cv::Rect
        bbox = cv::Rect(x, y, width, height);


            // 打印 BBox 信息
            // std::cout << "BBox: x=" << bbox.x << ", y=" << bbox.y
                    //   << ", width=" << bbox.width << ", height=" << bbox.height << std::endl;
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Unexpected number of bbox points: %ld", object.bbox_points.size());
        //     continue;
        // }

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

        // cv::resize(masks_instance, masks_instance, cv::Size(640, 480));

        // // 二值化图像，以确保只有黑白像素
        // cv::Mat binary_mask;
        // cv::threshold(masks_instance, binary_mask, 1, 255, cv::THRESH_BINARY);

        // // 查找轮廓，RETR_EXTERNAL表示只检测最外层的轮廓，CHAIN_APPROX_SIMPLE表示将轮廓压缩到最重要的点
        // std::vector<std::vector<cv::Point>> contours;
        // cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // // 初始化最大面积和对应的轮廓
        // double max_area = 0.0;
        // std::vector<cv::Point> largest_contour;

        // // 遍历所有找到的轮廓
        // for (const auto& contour : contours)
        // {
        //     double area = cv::contourArea(contour);  // 计算当前轮廓的面积
        //     if (area > max_area)  // 如果当前轮廓的面积大于最大面积，则更新最大面积和最大轮廓
        //     {
        //         max_area = area;
        //         largest_contour = contour;
        //     }
        // }


        // 找到所有轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(masks_instance, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 计算每个轮廓的凸包并找出最大凸包
        double max_area = 0.0;
        std::vector<cv::Point> largest_hull;

        // for (const auto& contour : contours)
        // {
        //     // 计算凸包
        //     std::vector<cv::Point> hull;
        //     cv::convexHull(contour, hull);

        //     // 计算凸包的面积
        //     double area = cv::contourArea(hull);
        //     if (area > max_area)
        //     {
        //         max_area = area;
        //         largest_hull = hull;
        //     }
        // }
        largest_hull = contours[0];

    
        // // 创建一个空白图像，用于绘制 maskpoints
        // cv::Mat output_image = cv::Mat::zeros(masks_instance.size(), CV_8UC3);

        // // 将每个轮廓绘制到 output_image 上
        // // for (const auto& contour : contours) {
        // //     cv::drawContours(output_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
        // // }

        // cv::drawContours(output_image, std::vector<std::vector<cv::Point>>{largest_hull}, -1, cv::Scalar(0, 255, 0), 2);
        
        // // 获取当前时间戳
        // auto now = std::chrono::system_clock::now();
        // auto in_time_t = std::chrono::system_clock::to_time_t(now);
        // auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // // 格式化时间戳为字符串
        // std::ostringstream oss;
        // oss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S") << '.' << std::setw(3) << std::setfill('0') << ms.count();
        // std::string timestamp = oss.str();

        // // 创建image文件夹，如果不存在的话
        // std::string folder_path = "./image";

        // // 生成文件名并将其放在image文件夹下
        // std::string filename = folder_path + "/image_" + timestamp + ".png";


        // // 生成唯一的文件名并保存图片
        // // std::string file_name = "output_mask_" + std::to_string(image_counter++) + ".png";
        // cv::imwrite(filename, output_image);

        // 将largest_hull 存入 DetectedObject
        ORB_SLAM3::DetectedObject detected_object(label, confidence, bbox, largest_hull);
        detected_objects.push_back(detected_object);

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

    // Sophus::SE3f Tcw = pSLAM_->TrackRGBD(imClahe, cv_ptrD->image, tImrgb);
    Sophus::SE3f Tcw = pSLAM_->TrackRGBD_yolo(imClahe, cv_ptrD->image, tImrgb, detected_objects);
    // std::cout<<"--------------------------size: "<<detected_objects.size()<<std::endl;
    detected_objects.clear();
    double dt = tImrgb - tlast;
    tlast = tImrgb;
    pose_.header.stamp = msgRGB->header.stamp;
    path_.header.stamp = msgRGB->header.stamp;
    trans_.header.stamp = msgRGB->header.stamp;

}


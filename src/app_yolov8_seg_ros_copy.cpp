// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include "opencv2/opencv.hpp"
// #include "./utils/yolo.h"
// #include "yolov8_seg.h"

// class YOLOv8Node : public rclcpp::Node {
// public:
//     YOLOv8Node() : Node("yolov8_node") {
//         // 初始化参数
//         setParameters(param_);

//         std::string model_path = "/home/nvidia/models/yolov8n-seg.trt";
//         std::vector<unsigned char> trt_file = utils::loadModel(model_path);
//         if (trt_file.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "trt_file is empty! Could not load file: %s", model_path.c_str());
//         } else if (!yolo_.init(trt_file)) {
//             RCLCPP_ERROR(this->get_logger(), "initEngine() occurred errors!");
//         }

//         yolo_.check();

//         // 创建订阅器
//         image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/camera/color/image_raw", 10, std::bind(&YOLOv8Node::imageCallback, this, std::placeholders::_1));

//         // 创建发布器
//         image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
//     }

//     void saveImages(const std::vector<cv::Mat>& imgsBatch, const std::string& directory) {
//     for (size_t i = 0; i < imgsBatch.size(); ++i) {
//         // 构造文件名
//         std::string filename = directory + "/image_" + std::to_string(i) + ".png";
        
//         // 保存图像到文件
//         if (!cv::imwrite(filename, imgsBatch[i])) {
//             std::cerr << "Error: Unable to save image to " << filename << std::endl;
//         }
//     }
// }

//     YOLOv8Seg::DetectionResult task(YOLOv8Seg& yolo_, const utils::InitParameter& param, std::vector<cv::Mat>& imgsBatch, const int& delayTime, const int& batchi)
//     {
//         yolo_.copy(imgsBatch);
//         utils::DeviceTimer d_t1; yolo_.preprocess(imgsBatch);  float t1 = d_t1.getUsedTime();
//         utils::DeviceTimer d_t2; yolo_.infer();				  float t2 = d_t2.getUsedTime();
//         utils::DeviceTimer d_t3; yolo_.postprocess(imgsBatch); float t3 = d_t3.getUsedTime();
//         float avg_times[3] = { t1 / param.batch_size, t2 / param.batch_size, t3 / param.batch_size };
//         sample::gLogInfo << "preprocess time = " << avg_times[0] << "; "
//             "infer time = " << avg_times[1] << "; "
//             "postprocess time = " << avg_times[2] << std::endl;

//         saveImages(imgsBatch, "/home/nvidia/yolov8-ros2/yolov8_ros/output");

//         YOLOv8Seg::DetectionResult result;
        
//         result = yolo_.showAndSave(param.class_names, delayTime, imgsBatch);
//         yolo_.reset();
//         return result;
//     }

// private:
//     void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//         cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        
//         std::vector<cv::Mat> imgs_batch = {frame};

//         YOLOv8Seg::DetectionResult result = task(yolo_, param_, imgs_batch, 1, 0);

//         auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", imgs_batch[0]).toImageMsg();
//         image_pub_->publish(*processed_msg);
//     }

//     void setParameters(utils::InitParameter& param) {
//         param.class_names = utils::dataSets::coco80;
//         param.num_class = 80; // for coco
//         param.batch_size = 1; // 设置 batch size 为 1 以便单张图像处理
//         param.dst_h = 640;
//         param.dst_w = 640;
//         param.input_output_names = {"images", "output0"};
//         param.conf_thresh = 0.25f;
//         param.iou_thresh = 0.7f;
//         param.save_path = "/home";
//     }

    
//     utils::InitParameter param_;
//     YOLOv8Seg yolo_{param_};

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<YOLOv8Node>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

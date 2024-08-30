#ifndef __RgbD_HEIGHT_NODE_HPP__
#define __RgbD_HEIGHT_NODE_HPP__
// #define WEBOTS
#define REALROBOTS
#define RgbdInertial 
// #define RGBD

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <boost/array.hpp>
#include <sophus/se3.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "height_estimator.hpp"

class RgbdInertialNode : public rclcpp::Node
{
public:
    RgbdInertialNode(ORB_SLAM3::System* pSLAM);
    ~RgbdInertialNode();

private:
    using ImuMsg = sensor_msgs::msg::Imu;
    using ImageMsg = sensor_msgs::msg::Image;
    const Eigen::Matrix3f R_ = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f t_ = Eigen::Vector3f::Zero();
    Sophus::SE3f Identity;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);
    void PubOdom(const Sophus::SE3f Twc, const double &dt);

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > subImgRgb_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > subImgDepth_;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    
    std::string rgb_topic_, depth_topic_, imu_topic_;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::TransformStamped trans_;
    nav_msgs::msg::Path path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ORB_SLAM3::System *pSLAM_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgRgbBuf_, imgDepthBuf_;
    std::mutex bufMutexRgb_, bufMutexDepth_;

    cv::Mat M1l_, M2l_, M1r_, M2r_;
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    bool clear_imu_point_flag_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    HeightEstimator *heightEstimator_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr heightPub_;
    void pubHeight(const Eigen::Matrix2d &heightMap);

    //camera params to declare
    int width_, height_, cellHeight_, cellWidth_, gridHeight_, gridWidth_;
};

#endif

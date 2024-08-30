#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__
// #define WEBOTS
#define REALROBOTS
#define StereoInertial
// #define Rgbd
// #define MovingAverageFilter
#define KalmanFilterImu
// #define rawImu

#define UDP_IP "127.0.0.1"
#define UDP_PORT 4197

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <boost/array.hpp>
#include <sophus/se3.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "kalman_filter_imu.hpp"

#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mutex>

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM);
    ~StereoInertialNode();

private:
    using ImuMsg = sensor_msgs::msg::Imu;
    using ImageMsg = sensor_msgs::msg::Image;
    Sophus::SE3f Identity;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    
    void GrabImu(const ImuMsg::SharedPtr msg);
    // void GrabImageLeft (const ImageMsg::SharedPtr& msg); // Grab Left Image
    // void GrabImageRight(const ImageMsg::SharedPtr& msg); // Grab Right Image
    void GrabStereo(const ImageMsg::SharedPtr imageLeft, const ImageMsg::SharedPtr imageRight); // Grab Stereo Images
    void PubOdom(const Sophus::SE3f Twc, const double &dt);     // Publish VIO Odom
    //void PubUdp(const double vx, const double vy, const double wz, const int sec, const int nsec);
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > subImageLeft_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > subImageRight_;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    std::string img_left_topic, img_right_topic, imu_topic_;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::TransformStamped trans_;
    nav_msgs::msg::Path path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ORB_SLAM3::System *pSLAM_;

    // IMU Buffer
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Stereo Buffer, not used.
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;
    size_t window_size_;
    std::deque<double> accel_x_, accel_y_, accel_z_;
    double sum_x_, sum_y_, sum_z_;
    std::shared_ptr<MultiKalmanFilter> kf_;

    cv::Mat M1l_, M2l_, M1r_, M2r_;
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    bool clear_imu_point_flag_;
    
    int sockfd;
    struct sockaddr_in servaddr;
};

#endif

#include "rgbd-inertial-node.hpp"
#include <chrono>

#include <opencv2/core/core.hpp>

using namespace std::placeholders;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System *pSLAM) :
    Node("ORB_SLAM3_ROS2"),
    pSLAM_(pSLAM),
    window_size_(5)
{
    // #define RgbdInertial
    // std::cout << "RGBD-Inertial node has been initialized!" << std::endl;
    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "camera/Rgb");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "camera/Depth");
    #ifdef RgbdInertial
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    // configure the collection window and publish period (default 1s)
    options.topic_stats_options.publish_period = std::chrono::seconds(1);

    // configure the topic name (default '/statistics')
    options.topic_stats_options.publish_topic = "/imu_topic_statistics";

    rmw_qos_profile_t imu_qos = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1000,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };

    // imu_topic_ = this->declare_parameter<std::string>("imu_topic", "imu");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "camera/Imu");
    cout<<"rgb_topic_: "<<rgb_topic_<<std::endl;
    cout<<"depth_topic_: "<<depth_topic_<<std::endl;
    cout<<"imu_topic: "<<imu_topic_<<std::endl;
    subImu_ = this->create_subscription<ImuMsg>(imu_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(imu_qos), imu_qos), std::bind(&RgbdInertialNode::GrabImu, this, _1), options);
    #endif
    subImgRgb_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, rgb_topic_);
    subImgDepth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, depth_topic_);
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *subImgRgb_, *subImgDepth_);
    syncApproximate->registerCallback(&RgbdInertialNode::GrabRGBD, this);
    // pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // pubPath_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);

    clear_imu_point_flag_ = true;
    std::cout << "RGBD-Inertial node has been initialized!" << std::endl;
    Identity = Sophus::SE3f(R_, t_);
    pose_.header.frame_id = "map";
    path_.header.frame_id = "map";
    trans_.header.frame_id = "odom";
    trans_.child_frame_id = "base_link";

    Eigen::VectorXd initial_value(3);
    initial_value << 0.0, -9.8, 0.0;
    kf_ = std::make_shared<MultiKalmanFilter>(1e-5, 0.0009939493244048429, 1.0, initial_value);

    // 创建套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    // 设置服务器地址和端口
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(UDP_IP);
    servaddr.sin_port = htons(UDP_PORT);
}

RgbdInertialNode::~RgbdInertialNode()
{
    // Stop all threads
    pSLAM_->Shutdown();

    // Save camera trajectory
    // pSLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}
#ifdef RgbdInertial
void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::cout << "GrabImu function called" << std::endl;

    #ifdef MovingAverageFilter
    // auto start = std::chrono::high_resolution_clock::now();
    // Add new data to deque
    accel_x_.push_back(msg->linear_acceleration.x);
    accel_y_.push_back(msg->linear_acceleration.y);
    accel_z_.push_back(msg->linear_acceleration.z);
    sum_x_ += msg->linear_acceleration.x;
    sum_y_ += msg->linear_acceleration.y;
    sum_z_ += msg->linear_acceleration.z;

    // Remove oldest data if deque size exceeds window size
    if (accel_x_.size() > window_size_) 
    {
        sum_x_ -= accel_x_.front();
        sum_y_ -= accel_y_.front();
        sum_z_ -= accel_z_.front();
        accel_x_.pop_front();
        accel_y_.pop_front();
        accel_z_.pop_front();
    }

    auto filtered_msg = std::make_shared<ImuMsg>(*msg);
    filtered_msg->linear_acceleration.x = sum_x_ / accel_x_.size();
    filtered_msg->linear_acceleration.y = sum_y_ / accel_y_.size();
    filtered_msg->linear_acceleration.z = sum_z_ / accel_z_.size();

    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // // 输出执行时间
    // std::cout << "Execution time: " << duration << " milliseconds" << std::endl;
    
    bufMutex_.lock();
    imuBuf_.push(filtered_msg);
    bufMutex_.unlock();
    // publisher_->publish(*filtered_msg);
    #endif
    #ifdef rawImu
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
    #endif
    #ifdef KalmanFilterImu
    Eigen::VectorXd measurement(3);
    measurement << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    
    Eigen::VectorXd filtered_measurement = kf_->update(measurement);

    auto filtered_msg = std::make_shared<ImuMsg>(*msg);
    filtered_msg->linear_acceleration.x = filtered_measurement(0);
    filtered_msg->linear_acceleration.y = filtered_measurement(1);
    filtered_msg->linear_acceleration.z = filtered_measurement(2);
    std::cout << "Filtered gyro x: " << filtered_msg->angular_velocity.x << "  y: "  << filtered_msg->angular_velocity.y << " z: "  << filtered_msg->angular_velocity.z<< std::endl;
    bufMutex_.lock();
    imuBuf_.push(filtered_msg);
    bufMutex_.unlock();
    #endif
}
#endif
void RgbdInertialNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD){
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

    #ifdef RgbdInertial
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    if (clear_imu_point_flag_)
        vImuMeas.clear();
    else
        clear_imu_point_flag_ = true;
    bufMutex_.lock();
    if (!imuBuf_.empty())
    {
        while (!imuBuf_.empty() && rclcpp::Time(imuBuf_.front()->header.stamp).seconds() <= tImrgb)
        {
            double t = rclcpp::Time(imuBuf_.front()->header.stamp).seconds();
            cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
            cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            imuBuf_.pop();
        }
    }
    else
        RCLCPP_ERROR(this->get_logger(), "No imu measurement");
    bufMutex_.unlock();
    if(vImuMeas.size() < 3)
    {
        RCLCPP_ERROR(this->get_logger(), "too less imu mea in vector, we get only %ld messages", vImuMeas.size());
        clear_imu_point_flag_ = false;
        return;
    }
    // else
    //     RCLCPP_INFO(this->get_logger(), "we get %ld imu messages", vImuMeas.size());
    #endif

    #ifdef RgbdInertial
    // Return T_cw
    Sophus::SE3f Tcw = pSLAM_->TrackRGBD(imClahe, cv_ptrD->image, tImrgb, vImuMeas);
    // Eigen::Vector3f vel = Tcw.so3() * pSLAM_->GetVel();
    // std::cout << "vel is  " << vel[0] << "_____" << vel[1] << "     "<< vel[2]<< std::endl;
    #endif
    #ifdef Rgbd
    Sophus::SE3f Tcw = pSLAM_->TrackRGBD(imClahe, cv_ptrD->image, tImrgb);
    #endif
    double dt = tImrgb - tlast;
    tlast = tImrgb;
    pose_.header.stamp = msgRGB->header.stamp;
    path_.header.stamp = msgRGB->header.stamp;
    trans_.header.stamp = msgRGB->header.stamp;
    // if(pSLAM_->IsBA2Finished())
    //     PubOdom(Tcw, dt);
}

// void RgbdInertialNode::PubOdom(const Sophus::SE3f Tcw, const double &dt)
// {
//     Eigen::Vector3f vel = Tcw.so3() * pSLAM_->GetVel();
//     static Sophus::SE3f prev_Twc = Identity;
//     Sophus::SE3f diff_Twc = prev_Twc.inverse() * Tcw.inverse();
//     prev_Twc = Tcw.inverse();

//     // 获取旋转矩阵
//     Eigen::Matrix3f Rcw_diff = diff_Twc.so3().matrix();
//     // Eigen::Matrix3f Rcw_diff = Rcw_diff.transpose();

//     double yaw = atan2(Rcw_diff(1, 0), Rcw_diff(0, 0));
//     double pitch = atan2(-Rcw_diff(2, 0), sqrt(Rcw_diff(2, 0) * Rcw_diff(2, 0) + Rcw_diff(2, 2) * Rcw_diff(2, 2))); // was R(2, 1) for the first term inside of sqrt
//     double roll = atan2(Rcw_diff(2, 1), Rcw_diff(2, 2));

//     // 获取 SE3f 对象中的平移向量 TODO:: is this covariance correct
//     Eigen::Vector3f translation_diff_vec = diff_Twc.translation();
//     double translation_diff = translation_diff_vec.norm();
//     double rotation_diff = acos((Rcw_diff(0, 0) + Rcw_diff(1, 1) + Rcw_diff(2, 2) - 1) / 2);

//     //计算动态协方差
//     //这里假设线速度和角速度的协方差与位姿的协方差相同
//     double translation_variance = std::max(0.1, 0.01 * translation_diff / dt);
//     double rotation_variance = std::max(0.001, 0.01 * Rcw_diff.norm()/ dt);

//     // 用计算的动态协方差替换原来的固定协方差
//     std::array<double, 36> pose_covariance = {
//                                                 translation_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
//                                                 0.0, translation_variance, 0.0, 0.0, 0.0, 0.0,
//                                                 0.0, 0.0, translation_variance, 0.0, 0.0, 0.0,
//                                                 0.0, 0.0, 0.0, rotation_variance, 0.0, 0.0,
//                                                 0.0, 0.0, 0.0, 0.0, rotation_variance, 0.0,
//                                                 0.0, 0.0, 0.0, 0.0, 0.0, rotation_variance
//                                             };

//     double angular_velocity_z = pitch / dt;

//     // 计算线速度和角速度
//     double linear_velocity_x = diff_Twc.translation().x() / dt;
//     double linear_velocity_y = diff_Twc.translation().y() / dt;
//     double linear_velocity_z = diff_Twc.translation().z() / dt;
//     // std::cout << "linear_velocity_x =  " << linear_velocity_x << std::endl;
//     // std::cout << "linear_velocity_y =  " << linear_velocity_y << std::endl;
//     // std::cout << "linear_velocity_z =  " << linear_velocity_z << std::endl;

//     #ifdef WEBOTS
//     double angle_rad = M_PI / 2;
//     Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
//     Eigen::Matrix3f Rwc = rotation_matrix  * Tcw.so3().matrix();
//     #endif
//     #ifdef REALROBOTS
//     double angle_rad = M_PI / 2;
//     Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(-angle_rad, Eigen::Vector3f::UnitY()).toRotationMatrix();
//     Eigen::Quaternionf quaternion(rotation_matrix);
//     Eigen::Matrix3f Rwc = Tcw.so3().matrix();
//     Eigen::Vector3f twc = Tcw.inverse().translation();
//     #endif

//     // 将旋转矩阵转换为四元数
//     // TODO: Check correctness of the pose
//     Eigen::Quaternionf qrot(Rwc);
//     Eigen::Quaternionf qtrans(qrot.w(), qrot.z(), qrot.x(), qrot.y());
//     Eigen::Quaternionf q = qtrans * quaternion;
//     // Eigen::Quaternionf q = qtrans;
//     auto msg = nav_msgs::msg::Odometry();
//     msg.header.stamp = pose_.header.stamp;;
//     msg.header.frame_id = "odom";
//     msg.child_frame_id = "base_link";
//     msg.pose.pose.position.x = twc(1);
//     msg.pose.pose.position.y = -twc(0);
//     msg.pose.pose.position.z = twc(2);
//     msg.pose.pose.orientation.x = q.x();
//     msg.pose.pose.orientation.y = q.y();
//     msg.pose.pose.orientation.z = q.z();
//     msg.pose.pose.orientation.w = q.w();
//     // 为 msg 消息设置线速度和角速度
//     msg.twist.twist.linear.x = linear_velocity_z;
//     msg.twist.twist.linear.y = linear_velocity_x;
//     msg.twist.twist.linear.z = 0.0;
//     msg.twist.twist.angular.z = angular_velocity_z;
//     msg.twist.twist.angular.x = 0.0;
//     msg.twist.twist.angular.y = 0.0;
//     msg.pose.covariance = pose_covariance;

//     pubOdom_->publish(msg);
//     pose_.pose = msg.pose.pose;
//     path_.poses.push_back(pose_);
//     pubPath_->publish(path_);

//     trans_.transform.translation.x = pose_.pose.position.x;
//     trans_.transform.translation.y = pose_.pose.position.y;
//     trans_.transform.translation.z = pose_.pose.position.z;

//     trans_.transform.rotation.x = pose_.pose.orientation.x;
//     trans_.transform.rotation.y = pose_.pose.orientation.y;
//     trans_.transform.rotation.z = pose_.pose.orientation.z;
//     trans_.transform.rotation.w = pose_.pose.orientation.w;
//     tf_broadcaster_->sendTransform(trans_);
//     // double stamp = rclcpp::Time(msg.header.stamp).seconds();
//     int ros_sec =  msg.header.stamp.sec;
//     int ros_nsec = msg.header.stamp.nanosec;
//     //  std::cout << "ros time: " << ros_sec << " " << ros_nsec << std::endl;

//     PubUdp(vel(2), vel(0), angular_velocity_z, ros_sec, ros_nsec);
// }

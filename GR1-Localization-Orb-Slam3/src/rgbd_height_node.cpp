#include "rgbd_height_node.hpp"
#include "height_estimator.hpp"
#include <opencv2/core/core.hpp>

using namespace std::placeholders;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System *pSLAM) :
    Node("ORB_SLAM3_ROS2"),
    pSLAM_(pSLAM)
{
    // std::cout << "RGBD-Inertial node has been initialized!" << std::endl;
    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "camera/Rgb");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "camera/Depth");
    gridWidth_ = this->declare_parameter<int>("gridWidth", 10);
    gridHeight_ = this->declare_parameter<int>("gridHeight", 10);

    #ifdef RgbdInertial
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "imu");
    subImu_ = this->create_subscription<ImuMsg>(imu_topic_, rclcpp::QoS(rclcpp::SystemDefaultsQoS()), std::bind(&RgbdInertialNode::GrabImu, this, _1));
    #endif
    subImgRgb_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, rgb_topic_);
    subImgDepth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, depth_topic_);
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *subImgRgb_, *subImgDepth_);
    syncApproximate->registerCallback(&RgbdInertialNode::GrabRGBD, this);
    pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pubPath_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);



    clear_imu_point_flag_ = true;
    std::cout << "RGBD-Inertial node has been initialized!" << std::endl;
    Identity = Sophus::SE3f(R_, t_);
    pose_.header.frame_id = "map";
    path_.header.frame_id = "map";
    trans_.header.frame_id = "map";
    trans_.child_frame_id = "odom";
    // RCLCPP_INFO(this->get_logger(), "Idasdsadsadsad");
    heightEstimator_ = new HeightEstimator;
    heightPub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("height", 10);
}

RgbdInertialNode::~RgbdInertialNode()
{
    // Stop all threads
    pSLAM_->Shutdown();
    delete heightEstimator_;

    // Save camera trajectory
    // pSLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}


#ifdef RgbdInertial
void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}
#endif


void RgbdInertialNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD){
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

    cv::Mat imRgb, imGray, depthRaw;
    try{
        #ifdef WEBOTS
        cv::cvtColor(cv_ptrRGB->image, imGray, cv::COLOR_BGRA2GRAY);
        #endif
        #ifdef REALROBOTS
        cv::cvtColor(cv_ptrRGB->image, imGray, cv::COLOR_RGB2GRAY);
        #endif
    }
    catch(cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
        return;
    }
    #ifdef RgbdInertial
    // Return T_cw
    // Process the depth image and estimate height
    depthRaw = cv_ptrD->image;
    Eigen::Matrix2d heightMap;
    heightEstimator_->processDepth(depthRaw, heightMap);

    Sophus::SE3f Tcw = pSLAM_->TrackRGBD(imGray, cv_ptrD->image, tImrgb, vImuMeas);
    #endif
    #ifdef RGBD
    Sophus::SE3f Tcw = pSLAM_->TrackRGBD(imGray, cv_ptrD->image, tImrgb);
    #endif
    double dt = tImrgb - tlast;
    tlast = tImrgb;
    pose_.header.stamp = msgRGB->header.stamp;
    path_.header.stamp = msgRGB->header.stamp;
    trans_.header.stamp = msgRGB->header.stamp;
    PubOdom(Tcw, dt);
}

void RgbdInertialNode::PubOdom(const Sophus::SE3f Tcw, const double &dt)
{
    static Sophus::SE3f prev_Tcw = Identity;
    Sophus::SE3f diff_Tcw = Tcw * prev_Tcw.inverse();
    prev_Tcw = Tcw;
    // RCLCPP_INFO(this->get_logger(), "diff_Twc: %f, %f, %f", diff_Twc.translation().x(), diff_Twc.translation().y(), diff_Twc.translation().z());
    // 获取旋转矩阵
    Eigen::Matrix3f Rcw_diff = diff_Tcw.so3().matrix();
    Eigen::Matrix3f Rwc_diff = Rcw_diff.transpose();

    double yaw = atan2(Rwc_diff(1, 0), Rwc_diff(0, 0));
    double pitch = atan2(-Rwc_diff(2, 0), sqrt(Rwc_diff(2, 0) * Rwc_diff(2, 0) + Rwc_diff(2, 2) * Rwc_diff(2, 2))); // was R(2, 1) for the first term inside of sqrt
    double roll = atan2(Rwc_diff(2, 1), Rwc_diff(2, 2));
    
    
    // 获取 SE3f 对象中的平移向量
    Eigen::Vector3f translation_diff_vec = diff_Tcw.translation();
    double translation_diff = translation_diff_vec.norm();
    double rotation_diff = acos((Rcw_diff(0, 0) + Rcw_diff(1, 1) + Rcw_diff(2, 2) - 1) / 2);

    //计算动态协方差
    //这里假设线速度和角速度的协方差与位姿的协方差相同
    double translation_variance = std::max(0.1, 0.01 * translation_diff / dt);
    double rotation_variance = std::max(0.001, 0.01 * Rcw_diff.norm()/ dt);


    // 计算线速度和角速度
    double linear_velocity_x = diff_Tcw.translation().x() / dt;
    double linear_velocity_y = diff_Tcw.translation().y() / dt;
    double linear_velocity_z = diff_Tcw.translation().z() / dt;
    double angular_velocity_x = roll / dt;
    double angular_velocity_y = pitch / dt;
    double angular_velocity_z = yaw / dt;

    // 用计算的动态协方差替换原来的固定协方差
    std::array<double, 36> pose_covariance = {
                                                translation_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, translation_variance, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, translation_variance, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, rotation_variance, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, rotation_variance, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, rotation_variance
                                            };
    // Translation in World frame
    Eigen::Vector3f tcw = Tcw.translation();
    #ifdef WEBOTS
    double angle_rad = 60 * M_PI / 180.0;
    Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::Matrix3f Rwc = Tcw.so3().matrix() * rotation_matrix ;
    Eigen::Vector3f twc =  rotation_matrix * tcw; // was translation_diff, causing twc around zero
    #endif
    #ifdef REALROBOTS
    Eigen::Matrix3f Rwc = Tcw.so3().matrix();
    Eigen::Vector3f twc = tcw;
    #endif

    // 将旋转矩阵转换为四元数
    // TODO: Check correctness of the pose
    Eigen::Quaternionf q(Rwc);
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = pose_.header.stamp;;
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = twc(1);
    msg.pose.pose.position.y = -twc(0);
    msg.pose.pose.position.z = twc(2);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    // 为 msg 消息设置线速度和角速度
    msg.twist.twist.linear.y = linear_velocity_y;
    msg.twist.twist.linear.z = linear_velocity_z;
    msg.twist.twist.angular.x = angular_velocity_x;
    msg.twist.twist.angular.y = angular_velocity_y;
    // 计算线速度和角速度的动态协方差
    double linear_velocity_x_variance = std::max(0.1, 0.01 * std::abs(linear_velocity_x));
    double linear_velocity_y_variance = std::max(0.1, 0.01 * std::abs(linear_velocity_y));
    double linear_velocity_z_variance = std::max(0.1, 0.01 * std::abs(linear_velocity_z));
    double angular_velocity_x_variance = std::max(0.001, 0.01 * std::abs(angular_velocity_x));
    double angular_velocity_y_variance = std::max(0.001, 0.01 * std::abs(angular_velocity_y));
    double angular_velocity_z_variance = std::max(0.001, 0.01 * std::abs(angular_velocity_z));

    std::array<double, 36> twist_covariance = {
                                        linear_velocity_x_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, linear_velocity_y_variance, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, linear_velocity_z_variance, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, angular_velocity_x_variance, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, angular_velocity_y_variance, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, angular_velocity_z_variance};
    
    //pose
    msg.pose.covariance = pose_covariance;

    // 计算速度
    double linear_velocity = translation_diff / dt;
    double angular_velocity = rotation_diff / dt;

    // 为 msg 消息设置线速度和角速度
    msg.twist.twist.linear.x = linear_velocity;
    msg.twist.twist.angular.z = angular_velocity;

    msg.twist.covariance = twist_covariance;

    pubOdom_->publish(msg);
    pose_.pose = msg.pose.pose;
    path_.poses.push_back(pose_);
    pubPath_->publish(path_);

    trans_.transform.translation.x = pose_.pose.position.x;
    trans_.transform.translation.y = pose_.pose.position.y;
    trans_.transform.translation.z = pose_.pose.position.z;

    trans_.transform.rotation.x = q.x();
    trans_.transform.rotation.y = q.y();
    trans_.transform.rotation.z = q.z();
    trans_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(trans_);
}

void RgbdInertialNode::pubHeight(const Eigen::Matrix2d &heightMap)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data.clear();

    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].size = heightMap.rows();
    msg.layout.dim[0].stride = heightMap.rows() * heightMap.cols();
    msg.layout.dim[0].label = "height";
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[1].size = heightMap.cols();
    msg.layout.dim[1].stride = heightMap.cols();
    msg.layout.dim[1].label = "width";
    
    for (int i = 0; i < heightMap.rows(); i++)
    {
        for (int j = 0; j < heightMap.cols(); j++)
        {
            msg.data.push_back(heightMap(i, j));
        }
    }
    heightPub_->publish(msg);
}
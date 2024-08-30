#include "rgbd-inertial-node.hpp"
// #include "stereo_inertial_node.hpp"
using json = nlohmann::json;

void RgbdInertialNode::PubUdp(const double vx, const double vy, const double wz, const int sec, const int nsec)
{
    json data;
    data["function"]="BroadcastCamera";
    data["data"]["vx"] = vx;
    data["data"]["vy"] = vy;
    data["data"]["wz"] = wz;
    data["data"]["sec"] = sec;
    data["data"]["nsec"] = nsec;
    std::string msg = data.dump();
    sendto(sockfd, msg.c_str(), msg.size(), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));
}

// void StereoInertialNode::PubUdp(const double vx, const double vy, const double wz, const int sec, const int nsec)
// {
//     json data;
//     data["vio_state"]["vx"] = vx;
//     data["vio_state"]["vy"] = vy;
//     data["vio_state"]["wz"] = wz;
//     data["vio_state"]["sec"] = sec;
//     data["vio_state"]["nsec"] = nsec;
//     std::string msg = data.dump();
//     sendto(sockfd, msg.c_str(), msg.size(), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));
// } 

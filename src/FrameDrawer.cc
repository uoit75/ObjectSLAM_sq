/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.



* Modification: Monocular Object SLAM Using Instance Segmentation with Superquadric Landmarks
* Version: 1.0
* Created: 2023/05/06
* Author: Shize Wang

*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include<opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ximgproc.hpp"



#include<mutex>


namespace ORB_SLAM3
{

cv::Scalar getColor(int id);

//[]TODO未移植
// 判断点是否在多边形内部
bool isPointInsidePolygon(const std::vector<cv::Point>& polygon, const cv::Point2f& point) {
    int crossings = 0;
    int n = polygon.size();

    for (int i = 0; i < n; i++) {
        cv::Point2f p1 = polygon[i];
        cv::Point2f p2 = polygon[(i + 1) % n];

        if (p1.y == p2.y)
            continue;

        if (point.y >= std::min(p1.y, p2.y) && point.y < std::max(p1.y, p2.y)) {
            double x = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;

            if (x > point.x)
                crossings++;
        }
    }

    return (crossings % 2 != 0);
}
// 将掩码信息解析为多边形坐标
std::vector<cv::Point2f> parseMask(const std::string& mask_str) {
    std::vector<cv::Point2f> polygon;
    std::istringstream iss(mask_str);
    float x, y;

    while (iss >> x >> y) {
        polygon.push_back(cv::Point2f(x, y));
    }

    return polygon;
}




FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    // if(state==Tracking::NOT_INITIALIZED)
    // {
    //     for(unsigned int i=0; i<vMatches.size(); i++)
    //     {
    //         if(vMatches[i]>=0)
    //         {
    //             cv::Point2f pt1,pt2;
    //             if(imageScale != 1.f)
    //             {
    //                 pt1 = vIniKeys[i].pt / imageScale;
    //                 pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
    //             }
    //             else
    //             {
    //                 pt1 = vIniKeys[i].pt;
    //                 pt2 = vCurrentKeys[vMatches[i]].pt;
    //             }
    //             cv::line(im,pt1,pt2,standardColor);
    //         }
    //     }
    //     for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
    //     {
    //         cv::Point2f pt1,pt2;
    //         if(imageScale != 1.f)
    //         {
    //             pt1 = (*it).first / imageScale;
    //             pt2 = (*it).second / imageScale;
    //         }
    //         else
    //         {
    //             pt1 = (*it).first;
    //             pt2 = (*it).second;
    //         }
    //         cv::line(im,pt1,pt2, standardColor,5);
    //     }

    // }
    // else if(state==Tracking::OK) //TRACKING
    // {
    //     mnTracked=0;
    //     mnTrackedVO=0;
    //     const float r = 5;
    //     int n = vCurrentKeys.size();
    //     for(int i=0;i<n;i++)
    //     {
    //         if(vbVO[i] || vbMap[i])
    //         {
    //             cv::Point2f pt1,pt2;
    //             cv::Point2f point;
    //             if(imageScale != 1.f)
    //             {
    //                 point = vCurrentKeys[i].pt / imageScale;
    //                 float px = vCurrentKeys[i].pt.x / imageScale;
    //                 float py = vCurrentKeys[i].pt.y / imageScale;
    //                 pt1.x=px-r;
    //                 pt1.y=py-r;
    //                 pt2.x=px+r;
    //                 pt2.y=py+r;
    //             }
    //             else
    //             {
    //                 point = vCurrentKeys[i].pt;
    //                 pt1.x=vCurrentKeys[i].pt.x-r;
    //                 pt1.y=vCurrentKeys[i].pt.y-r;
    //                 pt2.x=vCurrentKeys[i].pt.x+r;
    //                 pt2.y=vCurrentKeys[i].pt.y+r;
    //             }

    //             // This is a match to a MapPoint in the map
    //             if(vbMap[i])
    //             {
    //                 cv::rectangle(im,pt1,pt2,standardColor);
    //                 cv::circle(im,point,2,standardColor,-1);
    //                 mnTracked++;
    //             }
    //             else // This is match to a "visual odometry" MapPoint created in the last frame
    //             {
    //                 cv::rectangle(im,pt1,pt2,odometryColor);
    //                 cv::circle(im,point,2,odometryColor,-1);
    //                 mnTrackedVO++;
    //             }
    //         }
    //     }
    // }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

//[]
cv::Mat FrameDrawer::DrawFrame_yolo(float imageScale, YOLO trackedImageYolo)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    //[]
    vector<bool> vbObj;
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            //[]
            vbObj = mvbObj;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();


        //参数设置
        double rhoMin = 1.0f;	//最小长度
        double rhoMax = 360.0f;	//最大长度
        double rhoStep = 1;		//离散化单位长度
        double thetaMin = 0.0f;	//最小角度
        double thetaMax = CV_PI ;	//最大角度
        double thetaStep = CV_PI / 180.0f;		//离散化单位角度



        cout<<"trackedImageYolo.yoloLines.size() :"<<trackedImageYolo.yoloLines.size()<<endl;
        if(trackedImageYolo.yoloLines.size() > 0)
        {   
            
            //[]绘制bbox和掩码
            for(int i = 0; i<trackedImageYolo.yoloLines.size(); i++)
            {
                if(trackedImageYolo.yoloLines[i].cT <= 0.5)continue;
                drawPred(
                    stoi(trackedImageYolo.yoloLines[i].cla), 
                    trackedImageYolo.yoloLines[i].cT, 
                    trackedImageYolo.yoloLines[i].box.x - trackedImageYolo.yoloLines[i].box.width/2, 
                    trackedImageYolo.yoloLines[i].box.y - trackedImageYolo.yoloLines[i].box.height/2, 
                    trackedImageYolo.yoloLines[i].box.x + trackedImageYolo.yoloLines[i].box.width/2, 
                    trackedImageYolo.yoloLines[i].box.y + trackedImageYolo.yoloLines[i].box.height/2, 
                    im);

                if(trackedImageYolo.yoloLines[i].mask.size() > 0)
                {

                    //绘制掩码
                    fillPoly(im, trackedImageYolo.yoloLines[i].mask, getColor(stoi(trackedImageYolo.yoloLines[i].cla)), 8);

                }

            }
   


            for(int i=0;i<n;i++)
            {
                //如果这个点在视觉里程计中有(应该是追踪成功了的意思吧),在局部地图中也有
                if(vbVO[i] || vbMap[i])
                {
                    cv::Point2f pt1,pt2;
                    cv::Point2f point;

                    if(imageScale != 1.f)
                    {
                        point = vCurrentKeys[i].pt / imageScale;
                        float px = vCurrentKeys[i].pt.x / imageScale;
                        float py = vCurrentKeys[i].pt.y / imageScale;
                        pt1.x=px-r;
                        pt1.y=py-r;
                        pt2.x=px+r;
                        pt2.y=py+r;
                    }
                    else
                    {
                        point = vCurrentKeys[i].pt;
                        pt1.x=vCurrentKeys[i].pt.x-r;
                        pt1.y=vCurrentKeys[i].pt.y-r;
                        pt2.x=vCurrentKeys[i].pt.x+r;
                        pt2.y=vCurrentKeys[i].pt.y+r;
                    }


                    //[]绘制对象上的点
                    if(vbObj[i])
                    {
                        cv::rectangle(im,pt1,pt2,standardColor);
                        cv::circle(im,point,2,standardColor,-1);
                        mnTracked++;
                    }
                    else 
                    // else if(vbMap[i])
                    {
                        cv::rectangle(im,pt1,pt2,odometryColor);
                        cv::circle(im,point,2,odometryColor,-1);
                        mnTrackedVO++;
                    }
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);
    // for(int i = 0; i<trackedImageYolo.yoloLines.size(); i++)
    // {
    //     drawPred(
    //         stoi(trackedImageYolo.yoloLines[i].cla), 
    //         trackedImageYolo.yoloLines[i].cT, 
    //         trackedImageYolo.yoloLines[i].box.x, 
    //         trackedImageYolo.yoloLines[i].box.y, 
    //         trackedImageYolo.yoloLines[i].box.width, 
    //         trackedImageYolo.yoloLines[i].box.height, 
    //         im);
    // }
    // drawPred();


// void FrameDrawer::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame) 


    return imWithInfo;
}




cv::Mat FrameDrawer::DrawFrame_yolo_lines(float imageScale, YOLO trackedImageYolo, vector<cv::Vec4f> lines)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    //[]
    vector<bool> vbObj;
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            //[]
            vbObj = mvbObj;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();


        //参数设置
        double rhoMin = 1.0f;	//最小长度
        double rhoMax = 360.0f;	//最大长度
        double rhoStep = 1;		//离散化单位长度
        double thetaMin = 0.0f;	//最小角度
        double thetaMax = CV_PI ;	//最大角度
        double thetaStep = CV_PI / 180.0f;		//离散化单位角度


        //FLD法检测直线
        // Create FLD detector
        // Param               Default value   Description
        // length_threshold    10            - Segments shorter than this will be discarded
        // distance_threshold  1.41421356    - A point placed from a hypothesis line
        //                                     segment farther than this will be
        //                                     regarded as an outlier
        // canny_th1           50            - First threshold for
        //                                     hysteresis procedure in Canny()
        // canny_th2           50            - Second threshold for
        //                                     hysteresis procedure in Canny()
        // canny_aperture_size 3             - Aperturesize for the sobel
        //                                     operator in Canny()
        // do_merge            false         - If true, incremental merging of segments
        //                                     will be perfomred
        // int    length_threshold    = 20;
        // float  distance_threshold  = 1.41421356f;
        // double canny_th1           = 50.0;
        // double canny_th2           = 50.0;
        // int    canny_aperture_size = 3;
        // bool   do_merge            = true;
        // cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
        //         length_threshold,
        //         distance_threshold, 
        //         canny_th1, 
        //         canny_th2, 
        //         canny_aperture_size,
        //         do_merge);

        // // Convert the color image to grayscale
        // cv::Mat grayImage;
        // cv::cvtColor(im, grayImage, cv::COLOR_BGR2GRAY);
        // vector<cv::Vec4f> lines_fld;
        // fld->detect(grayImage, lines_fld);
        // cout<<"lines in framedrawer: "<<endl;
        // for(auto line : lines_fld)
        // {
        //     cout<<line<<endl;
        // }
   
        // cout<<"lines in tracking: "<<endl;
        // for(auto line : lines)
        // {
        //     cout<<line<<endl;
        // }


        cout<<"trackedImageYolo.yoloLines.size() :"<<trackedImageYolo.yoloLines.size()<<endl;
        if(trackedImageYolo.yoloLines.size() > 0)
        {   
            //[]绘制bbox和掩码
            for(int i = 0; i<trackedImageYolo.yoloLines.size(); i++)
            {
                
                drawPred(
                    stoi(trackedImageYolo.yoloLines[i].cla), 
                    trackedImageYolo.yoloLines[i].cT, 
                    trackedImageYolo.yoloLines[i].box.x - trackedImageYolo.yoloLines[i].box.width/2, 
                    trackedImageYolo.yoloLines[i].box.y - trackedImageYolo.yoloLines[i].box.height/2, 
                    trackedImageYolo.yoloLines[i].box.x + trackedImageYolo.yoloLines[i].box.width/2, 
                    trackedImageYolo.yoloLines[i].box.y + trackedImageYolo.yoloLines[i].box.height/2, 
                    im);

                if(trackedImageYolo.yoloLines[i].mask.size() > 0)
                {

                    //绘制掩码
                    // fillPoly(im, trackedImageYolo.yoloLines[i].mask, getColor(stoi(trackedImageYolo.yoloLines[i].cla)), 8);

                    // 只保留在掩码内部的线段
                    std::vector<cv::Vec4f> lines_in_mask;
                    for (const cv::Vec4f& line : lines) 
                    {
                        cv::Point2f p1(line[0], line[1]);
                        cv::Point2f p2(line[2], line[3]);

                        if (isPointInsidePolygon(trackedImageYolo.yoloLines[i].mask, p1) && isPointInsidePolygon(trackedImageYolo.yoloLines[i].mask, p2)) 
                        {
                            lines_in_mask.push_back(line);
                        }
                    }
                    // 在掩码内部绘制线段
                    int line_thickness = 3;

                    for (const cv::Vec4f& line : lines_in_mask) 
                    {
                        cv::Point pt1(line[0], line[1]);
                        cv::Point pt2(line[2], line[3]);
                        cv::line(im, pt1, pt2, getColor(stoi(trackedImageYolo.yoloLines[i].cla)), line_thickness);
                    }



                }

            }
   

            //绘制物体坐标轴
            for(int k = 0; k <= mpAtlas->GetCurrentMap()->MapObjects->Objects.size(); k++)
            {
            // cv::Mat R = mpAtlas->GetCurrentMap()->MapObjects->Objects[k].Twq(cv::Rect(0,0,3,3));
            // Eigen::Matrix<float, 3, 3>R_Eigen;
            // cv::cv2eigen(R, R_Eigen);
            
            //重力方向
            double angleG = mpAtlas->GetCurrentMap()->MapObjects->Objects[k].rotationX * M_PI / 180.0f;
            Eigen::Matrix3f RG_Eigen;
            RG_Eigen << 1, 0, 0,
                        0, cos(angleG), -sin(angleG),
                        0, sin(angleG), cos(angleG);

            Eigen::Matrix3f RR_Eigen;
            double angleR = mpAtlas->GetCurrentMap()->MapObjects->Objects[k].rotationY * M_PI / 180.0f;
            RR_Eigen << cos(angleR), 0, sin(angleR),
                        0, 1, 0,
                        -sin(angleR), 0, cos(angleR);

           
            // 定义坐标系的原点和坐标轴向量
            Eigen::Vector3f Obj3Dcenter = mpAtlas->GetCurrentMap()->MapObjects->Objects[k].Obj_Cen; 


            Eigen::Vector3f XAxis(Obj3Dcenter.x() + 0.1, Obj3Dcenter.y(), Obj3Dcenter.z());
            Eigen::Vector3f YAxis(Obj3Dcenter.x(), Obj3Dcenter.y() + 0.1, Obj3Dcenter.z());
            Eigen::Vector3f ZAxis(Obj3Dcenter.x(), Obj3Dcenter.y(), Obj3Dcenter.z() + 0.1);


            Eigen::Vector3f XAxis_R = RG_Eigen * RR_Eigen * (XAxis - Obj3Dcenter) + Obj3Dcenter;
            Eigen::Vector3f YAxis_R = RG_Eigen * RR_Eigen * (YAxis - Obj3Dcenter) + Obj3Dcenter;
            Eigen::Vector3f ZAxis_R = RG_Eigen * RR_Eigen * (ZAxis - Obj3Dcenter) + Obj3Dcenter;


            Eigen::Vector2f eigencenter = currentFrame.ProjectObjCenter(Obj3Dcenter);
            Eigen::Vector2f eigenXAxis = currentFrame.ProjectObjCenter(XAxis_R);
            Eigen::Vector2f eigenYAxis = currentFrame.ProjectObjCenter(YAxis_R);
            Eigen::Vector2f eigenZAxis = currentFrame.ProjectObjCenter(ZAxis_R);


            cv::Point2f Obj2Dcenter(eigencenter.x(), eigencenter.y());
            cv::Point2f imgXAxis(eigenXAxis.x(), eigenXAxis.y());
            cv::Point2f imgYAxis(eigenYAxis.x(), eigenYAxis.y());
            cv::Point2f imgZAxis(eigenZAxis.x(), eigenZAxis.y());


            // 在图像上绘制坐标系
            cv::line(im, Obj2Dcenter, imgXAxis, cv::Scalar(0, 0, 255), 3); // 绘制X轴为红色
            cv::line(im, Obj2Dcenter, imgYAxis, cv::Scalar(0, 255, 0), 3); // 绘制Y轴为绿色
            cv::line(im, Obj2Dcenter, imgZAxis, cv::Scalar(255, 0, 0), 3); // 绘制Z轴为蓝色
            }


            for(int i=0;i<n;i++)
            {
                //如果这个点在视觉里程计中有(应该是追踪成功了的意思吧),在局部地图中也有
                if(vbVO[i] || vbMap[i])
                {
                    cv::Point2f pt1,pt2;
                    cv::Point2f point;

                    if(imageScale != 1.f)
                    {
                        point = vCurrentKeys[i].pt / imageScale;
                        float px = vCurrentKeys[i].pt.x / imageScale;
                        float py = vCurrentKeys[i].pt.y / imageScale;
                        pt1.x=px-r;
                        pt1.y=py-r;
                        pt2.x=px+r;
                        pt2.y=py+r;
                    }
                    else
                    {
                        point = vCurrentKeys[i].pt;
                        pt1.x=vCurrentKeys[i].pt.x-r;
                        pt1.y=vCurrentKeys[i].pt.y-r;
                        pt2.x=vCurrentKeys[i].pt.x+r;
                        pt2.y=vCurrentKeys[i].pt.y+r;
                    }


                    //[]绘制对象上的点
                    if(vbObj[i])
                    {
                        cv::rectangle(im,pt1,pt2,standardColor);
                        cv::circle(im,point,2,standardColor,-1);
                        mnTracked++;
                    }
                    else 
                    // else if(vbMap[i])
                    {
                        cv::rectangle(im,pt1,pt2,odometryColor);
                        cv::circle(im,point,2,odometryColor,-1);
                        mnTrackedVO++;
                    }
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);
    // for(int i = 0; i<trackedImageYolo.yoloLines.size(); i++)
    // {
    //     drawPred(
    //         stoi(trackedImageYolo.yoloLines[i].cla), 
    //         trackedImageYolo.yoloLines[i].cT, 
    //         trackedImageYolo.yoloLines[i].box.x, 
    //         trackedImageYolo.yoloLines[i].box.y, 
    //         trackedImageYolo.yoloLines[i].box.width, 
    //         trackedImageYolo.yoloLines[i].box.height, 
    //         im);
    // }
    // drawPred();


// void FrameDrawer::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame) 


    return imWithInfo;
}



cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im,pt1,pt2,cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x=mvCurrentKeysRight[i].pt.x-r;
                    pt1.y=mvCurrentKeysRight[i].pt.y-r;
                    pt2.x=mvCurrentKeysRight[i].pt.x+r;
                    pt2.y=mvCurrentKeysRight[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,point,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,point,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}



void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mThDepth = pTracker->mCurrentFrame.mThDepth;
    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    //[]
    mvbObj = vector<bool>(N,false);

    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
                //[]
                if(pMP->whetherInObject)
                {
                    mvbObj[i] = true;
                }
                else
                {
                    mvbObj[i] = false;
                }
            }
        }

    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}


void FrameDrawer::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)   // Draw the predicted bounding box
{
	//Draw a rectangle displaying the bounding box
	cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 1);

	//Get the label for the class name and its confidence
	// string label = cv::format("%.2f", conf);
	// label = classId + ":" + label;

    string label = to_string(classId);
	label = label + ":" + to_string(conf);

	//Display the label at the top of the bounding box
	int baseLine;
	cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	//rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
	cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 1);
}





cv::Scalar getColor(int id)
{
    // 使用取余操作循环选择颜色
    int colorCount = 20; // 颜色总数
    int index = id % colorCount; // 计算索引，从0开始

    switch (index)
    {
        case 0:
            return cv::Scalar(210,116,23);
        case 1:
            return cv::Scalar(169,169,169);
        case 2:
            return cv::Scalar(30,105,210);
        case 3:
            return cv::Scalar(0,128,128);
        case 4:
            return cv::Scalar(230,216,173);
        case 5:
            return cv::Scalar(130,0,75);
        case 6:
            return cv::Scalar(203,192,255);
        case 7:
            return cv::Scalar(0,0,139);
        case 8:
            return cv::Scalar(10,215,255);
        case 9:
            return cv::Scalar(144,238,144);
        case 10:
            return cv::Scalar(130,0,75);
        case 11:
            return cv::Scalar(147,20,255);
        case 12:
            return cv::Scalar(0,69,255);
        case 13:
            return cv::Scalar(0,255,255);
        case 14:
            return cv::Scalar(139,139,0);
        case 15:
            return cv::Scalar(128,0,128);
        case 16:
            return cv::Scalar(238,130,238);
        case 17:
            return cv::Scalar(129,119,69);
        case 18:
            return cv::Scalar(70,135,230);
        case 19:
            return cv::Scalar(124,138,104);
                   
    }
    // if(id >= 0&& id < 20)
    //     return cv::Scalar(id, 2*id, id);
    // else if(id >= 20 && id < 40)
    //     return cv::Scalar(2*id, id, 0);
    // else if(id >= 40 && id < 60)
    //     return cv::Scalar(3*id, 2*id, id);
    // else 
    //     return cv::Scalar(id, 0, id);


    
}





} //namespace ORB_SLAM

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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <string>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

//[]读取yolotext
void readTextFiles(const std::string& folderPath,const std::string& imgname, std::vector<ORB_SLAM3::TextLine>& lines);


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);



int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/timestamp.txt";

    LoadImages(strFile, vstrImageFilenames, vTimestamps);


    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_resize = 0.f;
    double t_track = 0.f;


    // []Main loop, include read text
    cv::Mat im;
    std::vector<ORB_SLAM3::TextLine> lines;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+vstrImageFilenames[ni]+".png",cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << vstrImageFilenames[ni]+".png" << endl;
            return 1;
        }


        //  cv::resize(im, im, cv::Size(612, 512));
         cv::resize(im, im, cv::Size(640, 480));

// #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif


        //[]读取yolotxt
        readTextFiles(string(argv[3])+"labels/", std::to_string(vTimestamps[ni]), lines);
        if(lines.empty())
        {
            cerr << endl << "Failed to load text"<< endl;
            // return 1;
        }
        // cout<<"a image----------------------------------"<<endl;
        // for (const auto& line : lines) {
        //         std::cout << line.text << std::endl;
        // }

        // cout<<"lines init:"<<lines.size()<<endl;
        // Pass the image to the SLAM system
        SLAM.TrackMonocular_yolo(im,tframe,lines);
        lines.clear();

// #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
        // std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    std::this_thread::sleep_for(std::chrono::seconds(3000));

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //[]
    SLAM.SaveMappointPose("ObjectMappointPose.txt");
    SLAM.SaveMappointDistance("ObjectMappointDistance.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(to_string(t));
        }
    }
}

//[]读取yolotext
void readTextFiles(const std::string& folderPath,const std::string& imgname, std::vector<ORB_SLAM3::TextLine>& lines) {
    
        std::string filePath = folderPath + imgname + ".txt";
        std::ifstream file(filePath);
        if (file.is_open()) {
            std::string line;
            while (getline(file, line)) {
                lines.push_back(ORB_SLAM3::TextLine(line));
            }
            file.close();
        }
    
}
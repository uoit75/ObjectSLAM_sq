/*

* Modification: Monocular Object SLAM Using Instance Segmentation with Superquadric Landmarks
* Version: 1.0
* Created: 2023/05/06
* Author: Shize Wang

*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sys/wait.h>
#include<unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <dirent.h>
#include <string>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames);
// string GetDatasetName(const string &strSequencePath);
void readTextFiles(const std::string& folderPath,const std::string& imgname, std::vector<ORB_SLAM3::TextLine>& lines);


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }


    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    string strFile = string(argv[3])+"/color";
    LoadImages(strFile, vstrImageFilenames);


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



   // Main loop
    cv::Mat im;
    std::vector<ORB_SLAM3::TextLine> lines;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/color/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        cv::resize(im, im, cv::Size(640, 480));

        double tframe = ni;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << strFile << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        
        //[]读取yolotxt

        // 找到最后一个斜杠的位置
        size_t lastSlash = vstrImageFilenames[ni].find_last_of('/');

        // 获取文件名（包括文件扩展名）部分
        std::string fileNameWithExtension = vstrImageFilenames[ni].substr(lastSlash + 1);

        // 将文件扩展名修改为 ".txt"
        size_t dotPosition = fileNameWithExtension.find_last_of('.');
        std::string fileNameWithoutExtension = fileNameWithExtension.substr(0, dotPosition);
        std::string newFileName = fileNameWithoutExtension + ".txt";

        readTextFiles(string(argv[3])+"labels/", newFileName, lines);
        if(lines.empty())
        {
            cerr << endl << "Failed to load text"<< endl;
            cout<<string(argv[3])+"labels/"<<newFileName<<endl;
            // return 1;
        }
        SLAM.TrackMonocular_yolo(im,tframe,lines);
        lines.clear();

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;


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









  
void ls(const string &path, vector<string> &vstrImageFilenames) {
    DIR *mydir;
    struct dirent *myfile;
    struct stat mystat;

    mydir = opendir(path.c_str());
    if (!mydir) {
        cerr << "Unable to open " << path << endl;
        exit(1);
    }
    while((myfile = readdir(mydir)) != NULL)
    {
        stat(myfile->d_name, &mystat); 
        std::string filename = std::string(myfile->d_name);
        if (filename != "." && filename != "..")
            vstrImageFilenames.push_back(filename);
    }
    closedir(mydir);
    
    sort(vstrImageFilenames.begin(), 
        vstrImageFilenames.end(), 
        [](string a, string b) {
            return stoi(a) < stoi(b);
    });
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames)
{
    ls(strFile, vstrImageFilenames);
}


//[]读取yolotext
void readTextFiles(const std::string& folderPath,const std::string& imgname, std::vector<ORB_SLAM3::TextLine>& lines) 
{
    
        std::string filePath = folderPath + imgname;
        std::ifstream file(filePath);
        if (file.is_open()) {
            std::string line;
            while (getline(file, line)) {
                lines.push_back(ORB_SLAM3::TextLine(line));
            }
            file.close();
        }
    
}
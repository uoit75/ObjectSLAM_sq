/**

* Version: 1.0
* Created: 2023/05/06
* Author: Shize Wang

*/


#ifndef OBJECT_H
#define OBJECT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "MapPoint.h"




namespace ORB_SLAM3
{

class MapPoint;
class Frame;
class YOLO;

double CalculateAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& q1, const cv::Point& q2);
bool areLinesParallel(const cv::Vec4f& line1, const cv::Vec4f& line2) ;


    class KMalgorithm
    {
    public:
        // int MAXN = 305;
        // int INF = 0x3f3f3f3f;
        //男生是mapobj，女生是curobj

        // int love[currentObjects.Objects.size()][AllObjects.Objects.size()];   // 记录每个妹子和每个男生的好感度
        std::vector<std::vector<int>> love;// 记录每个妹子和每个男生的好感度
        std::vector<int> ex_Curobj;// 每个妹子的期望值
        std::vector<int> ex_Mapobj;// 每个男生的期望值
        std::vector<bool> vis_Curobj;// 记录每一轮匹配匹配过的女生
        std::vector<bool> vis_Mapobj;// 记录每一轮匹配匹配过的男生
        std::vector<int> match;// 记录每个男生匹配到的妹子 如果没有则为-1  记录每个妹子匹配到的男生 如果没有则为-1
        std::vector<int> slack;// 记录每个汉子如果能被妹子倾心最少还需要多少期望值

        
        KMalgorithm(int CurObjsize, int MapObjsize)
        {
            ex_Curobj.resize(CurObjsize);
            ex_Mapobj.resize(MapObjsize);
            vis_Curobj.resize(CurObjsize);
            vis_Mapobj.resize(MapObjsize);
            match.resize(MapObjsize);
            slack.resize(MapObjsize);
            love.resize(CurObjsize, std::vector<int>(MapObjsize, 0));
        }

        
    };


    // BRIEF the object in current frame.
    class Object
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        class Object_3D
        {
            public:
                int mnId;//物体自身指定id
                int cid;      // class id.
                float Score = 0;       // Probability.

                float length = 0;
                float width = 0;
                float height = 0;

                //重力方向rotationX
                float rotationX = 20;
                float rotationY = 0;
                float rotationZ = 0;

                float e1 = 1;
                float e2 = 1;

                float mean = 0;//均值
                float SD = 0;//标准差

                float meanX = 0;//均值
                float SDX = 0;//标准差
                float meanY = 0;//均值
                float SDY = 0;//标准差
                float meanZ = 0;//均值
                float SDZ = 0;//标准差

                bool WhetherInFrame = false;

                //定义变换矩阵
                cv::Mat Twq = (cv::Mat_<float>(4,4)<<   1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0,
                                                        0, 0, 0, 1);



                vector<cv::KeyPoint>  Obj_KPs;
                vector<MapPoint*>  Obj_MPs;  
                Eigen::Vector3f  Obj_Cen;
                cv::Rect Obj_box;
                // mBoxRect = cv::Rect(box.x, box.y, box.width, box.height);

                //观测到物体的帧
                vector<int>Obj_Fms;
                int FrameId;
                Eigen::Vector3f FramePose;

                //检测到的直线矩阵
                cv::Mat lines;
                
                //储存掩码信息
                vector<cv::Point> mask;


  

        Object_3D(){};
        Object_3D(int class_id, vector<cv::KeyPoint> Obj_KeyPoints, vector<MapPoint*>  Obj_MapPoints) :cid(class_id), Obj_KPs(Obj_KeyPoints), Obj_MPs(Obj_MapPoints), mnId(0){}               
        ~Object_3D(){};
        };
        

    

    vector<Object_3D> Objects;
    Object(){};
    Object(vector<int>cidVec, vector<vector<cv::KeyPoint>>Obj_KPsVec, vector<vector<MapPoint*>>Obj_MPsVec)
    {
        for (int i = 0; i < Obj_MPsVec.size(); i++) 
        {
            // if(Obj_MPsVec[i].size() < 10)continue;//地图点小于10不初始化
            Objects.push_back(Object_3D(cidVec[i], Obj_KPsVec[i], Obj_MPsVec[i]));
        }
    }



    void printObject() 
    {
        for (int i = 0; i < Objects.size(); i++) 
        {
            cout << "Objects cid: ";

                cout << Objects[i].cid << " ";
            
            cout << endl << "Objects KeyPoints size: ";

                cout << Objects[i].Obj_KPs.size() << endl;
            
        }
    }

    static Object InitialObject(Frame currentFrame, YOLO currentYolo);
    static bool InPolygon (cv::Point p, vector<cv::Point>& Polygon);  //没用
    void ComputeObjectCenter();
    static void Association(Object currentObjects, Object& AllObjects, Object& newobjects);
    static void AssociateObject(Object::Object_3D currentObject, Object::Object_3D& MapObject);
    static void CommputeDistanceSD(Object::Object_3D& MapObject);
    static void CommputeDistanceSDXYZ(Object::Object_3D& Object);
    static void CommputeDDistanceSD(Object::Object_3D& Object);
    static void RemoveOutlier(Object &Objs);
    static void RemoveOutlierXYZ(Object &Objs);
    static void RemoveOutlierD(Object_3D &Obj, float SDnumber);
    static void IndependentTtest(Object &MPObjs, Object &Objs);
    static void IndependentDistanceTtest(Object &MPObjs);
    static double WhetherInSuperquadric(Object::Object_3D currentObject, Object::Object_3D MapObject);
    static float Calculate3DIoU(Object::Object_3D currentObject, Object::Object_3D MapObject);
    static void KMassociation(Object currentObjects, Object& AllObjects, Object& newobjects);
    static bool dfs(int Curobj, KMalgorithm &KM, int count);
    static void KMcalculate(KMalgorithm &KM);
    static void ComputeRotation(Object::Object_3D& Object, Frame currentFrame, vector<cv::Vec4f> lines);

    };













} //namespace ORB_SLAM

#endif // OBJECT_H
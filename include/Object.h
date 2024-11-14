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

//////////DBSCAN
#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define DBSCAN_FAILURE -3
//////////DBSCAN



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

        std::vector<std::vector<int>> love;
        std::vector<int> ex_Curobj;
        std::vector<int> ex_Mapobj;
        std::vector<bool> vis_Curobj;
        std::vector<bool> vis_Mapobj;
        std::vector<int> match;
        std::vector<int> slack;

        
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


    class Object
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        class Object_3D
        {
            public:
                int mnId;
                int cid;      
                float Score = 0;      

                float length = 0;
                float width = 0;
                float height = 0;

                float rotationX = 20;//for TUM dataset gravity
                float rotationY = 0;
                float rotationZ = 0;

                float e1 = 1;
                float e2 = 1;

                float mean = 0;
                float SD = 0;

                float meanX = 0;
                float SDX = 0;
                float meanY = 0;
                float SDY = 0;
                float meanZ = 0;
                float SDZ = 0;

                bool WhetherInFrame = false;

               
                cv::Mat Twq = (cv::Mat_<float>(4,4)<<   1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0,
                                                        0, 0, 0, 1);



                vector<cv::KeyPoint>  Obj_KPs;
                vector<MapPoint*>  Obj_MPs;  
                Eigen::Vector3f  Obj_Cen;
                cv::Rect Obj_box;

                vector<int>Obj_Fms;
                int FrameId;
                Eigen::Vector3f FramePose;

   
                cv::Mat lines;
 
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
    static bool InPolygon (cv::Point p, vector<cv::Point>& Polygon);  
    void ComputeObjectCenter();
    static void Association(Object currentObjects, Object& AllObjects, Object& newobjects);
    static void AssociateObject(Object::Object_3D currentObject, Object::Object_3D& MapObject);
    static void CommputeDistanceSD(Object::Object_3D& MapObject);
    static void CommputeDistanceSDXYZ(Object::Object_3D& Object);
    static void CommputeDDistanceSD(Object::Object_3D& Object);
    static void RemoveOutlier(Object &Objs);
    static void RemoveOutlierXYZ(Object &Objs);
    static void RemoveOutlierD(Object_3D &Obj, float SDnumber);
    static void RemoveOutlierDBSCAN(Object_3D &Obj, float MINIMUM_POINTS, float EPSILON);
    static void IndependentTtest(Object &MPObjs, Object &Objs);
    static void IndependentDistanceTtest(Object &MPObjs);
    static double WhetherInSuperquadric(Object::Object_3D currentObject, Object::Object_3D MapObject);
    static float Calculate3DIoU(Object::Object_3D currentObject, Object::Object_3D MapObject);
    static void KMassociation(Object currentObjects, Object& AllObjects, Object& newobjects);
    static bool dfs(int Curobj, KMalgorithm &KM, int count);
    static void KMcalculate(KMalgorithm &KM);
    static void ComputeRotation(Object::Object_3D& Object, Frame currentFrame, vector<cv::Vec4f> lines);


        class DBSCAN 
        {
        public:    
            DBSCAN(unsigned int minPts, float eps, vector<cv::Point3f> points)
                : m_points(points), m_minPoints(minPts), m_epsilon(eps), m_pointSize(points.size())
            {
                clusterIDs.resize(m_pointSize, UNCLASSIFIED);
            }

            int run();
            vector<int> calculateCluster(cv::Point3f point);
            int expandCluster(int pointIndex, int clusterID);
            double calculateDistance(const cv::Point3f& pointCore, const cv::Point3f& pointTarget);

            int getTotalPointSize() { return m_pointSize; }
            int getMinimumClusterSize() { return m_minPoints; }
            int getEpsilonSize() { return m_epsilon; }

        public:
            vector<cv::Point3f> m_points;
            vector<int> clusterIDs; 
            unsigned int m_pointSize;
            unsigned int m_minPoints;
            float m_epsilon;
        };



    };














} //namespace ORB_SLAM

#endif // OBJECT_H
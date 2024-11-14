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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>




namespace ORB_SLAM3
{


cv::Scalar getColorInmap(int id);
void cvScalarToGLColor3f(const cv::Scalar& scalar, GLfloat& red, GLfloat& green, GLfloat& blue);
// std::vector<Vertex> calculateHyperQuadricVertices(float a, float b, float c, const Eigen::Matrix4f& rotationMatrix);
Eigen::Vector3d SuperEllipsoidCurve(double theta1,double theta2,double p1,double p2);





MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

//[superquadric]
void MapDrawer::DrawMapPoints_background()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;
    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();


    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        if(vpMPs[i]->whetherInObject == true)
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();


    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if((*sit)->whetherInObject == true)
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));

    }
    glEnd();

}


void MapDrawer::DrawMapPoints_objects()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();

    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();


    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(4);
    glBegin(GL_POINTS);
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if((*sit)->whetherInObject == false)
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();

        cv::Scalar color = getColorInmap((*sit)->ObjectId);
        GLfloat red, green, blue;
        cvScalarToGLColor3f(color, red, green, blue);
        glColor3f(red, green, blue);

        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(4);
    glBegin(GL_POINTS);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        if(vpMPs[i]->whetherInObject == false)
            continue;

        cv::Scalar color = getColorInmap(vpMPs[i]->ObjectId); 
        GLfloat red, green, blue;
        cvScalarToGLColor3f(color, red, green, blue); 
        glColor3f(red, green, blue);

        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

}




void MapDrawer::DrawMapPoints_ObjectCenters()
{
  
    glPointSize(8);
    glBegin(GL_POINTS);
    glColor3f(1,0.5,0.0);

    if(mpAtlas->GetCurrentMap()->MapObjects->Objects.size() == 0)return;
    
    for(size_t i=0, iend=mpAtlas->GetCurrentMap()->MapObjects->Objects.size(); i<iend;i++)
    {
        Eigen::Matrix<float,3,1> pos = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].Obj_Cen;
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

}



//[superquadric]
void MapDrawer::DrawObjectSuperQuadrics()
{
#define PI 3.14159265358979323846
#define TWOPI 2*3.14159265358979323846
#define PID2 3.14159265358979323846/2.0
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

if(mpAtlas->GetCurrentMap()->MapObjects->Objects.size() == 0)return;

for(int i = 0; i < mpAtlas->GetCurrentMap()->MapObjects->Objects.size(); i++)
{
    // Define superellipsoid parameters
    float rx = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].length;   // x-radius
    float ry = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].width;   // y-radius
    float rz = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].height;   // z-radius
    float e1 ;  // first exponent
    float e2 ;  // second exponent

    e1 = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].e1;
    e2 = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].e2;

    // Define resolution
    int res = 30;
    GLuint  superquadric = 0;

    Eigen::Vector3d x(0, 0, 0);
    double theta3;

    // Draw superellipsoid
    glPushMatrix();
    superquadric  = glGenLists(1);
    glNewList( superquadric, GL_COMPILE );
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    Eigen::Vector3f ObjCenter = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].Obj_Cen;

    //Assume the direction of gravity.
    // Step 1: Translate the scene so that the center of mass of the object is at the origin (0, 0, 0).
    glTranslatef(ObjCenter[0], ObjCenter[1], ObjCenter[2]);
    // Step 2: Define the overall rotation angle.
    double rotationX = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].rotationX;
    glRotatef(rotationX, 1.0f, 0.0f, 0.0f);
    glRotatef(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].rotationY, 0.0f, 1.0f, 0.0f);


    // Step 3: Translate the scene again to restore the point to its original position.
    glTranslatef(-ObjCenter[0], -ObjCenter[1], -ObjCenter[2]);



    cv::Scalar color = getColorInmap(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].mnId); 
   
    GLfloat red, green, blue;
    cvScalarToGLColor3f(color, red, green, blue); 
    glColor3f(red, green, blue);


    glTranslatef(ObjCenter[0], ObjCenter[1], ObjCenter[2]);

    glScalef((GLfloat)(rx), (GLfloat)(ry), (GLfloat)(rz));



    //Loop to draw the surface of a superquadric.
    double dt = 0.01 * TWOPI / res;
    for (int j=0;j<res/2;j++) //Vertical partitioning
    {
        double theta1 = j * TWOPI / (double)res - PID2;
        double theta2 = (j + 1) * TWOPI / (double)res - PID2;

        // Set line width.
        glLineWidth(1.0f);

        glBegin(GL_QUAD_STRIP);

        for (int i=0;i<=res;i++) // Horizontal partitioning
        {

            if (i == 0 || i == res)
                theta3 = 0;
            else
                theta3 = i * TWOPI / res;


            x = SuperEllipsoidCurve(theta2,theta3,e1,e2);
            glVertex3f(x[0],x[1],x[2]);


            x = SuperEllipsoidCurve(theta1,theta3,e1,e2);
            glVertex3f(x[0],x[1],x[2]);


        }
        glEnd();
    }



    glEndList();
    glCallList(superquadric);  //draw
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);  

    glPopMatrix();

}

}





void MapDrawer::DrawMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
 
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        if(vpMPs[i]->whetherInObject == true)
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if((*sit)->whetherInObject == true)
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));

    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if((*sit)->whetherInObject == false)
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        if(vpMPs[i]->whetherInObject == false)
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();


}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat*)Twc.data());

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
                    }
                    else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f,0.0f,1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f,0.0f,1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && pActiveMap->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i<4; i++) {
        M.m[4*i] = Twc(0,i);
        M.m[4*i+1] = Twc(1,i);
        M.m[4*i+2] = Twc(2,i);
        M.m[4*i+3] = Twc(3,i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0,3);
    MOw.m[13] = Twc(1,3);
    MOw.m[14] = Twc(2,3);
}




cv::Scalar getColorInmap(int id)
{
    
    // Use modulo operation to cycle through colors.
    int colorCount = 20; 
    int index = id % colorCount; 

    switch (index)
    {
        case 0:
            return cv::Scalar(10,215,255);
        case 1:
            return cv::Scalar(130,0,75);
        case 2:
            return cv::Scalar(230,216,173);
        case 3:
            return cv::Scalar(0,128,128);
        case 4:
            return cv::Scalar(144,238,144);
        case 5:
            return cv::Scalar(130,0,75);
        case 6:
            return cv::Scalar(203,192,255);
        case 7:
            return cv::Scalar(0,0,139);
        case 8:
            return cv::Scalar(210,116,23);
        case 9:
            return cv::Scalar(169,169,169);
        case 10:
            return cv::Scalar(30,105,210);
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
            return cv::Scalar(70,135,280);
        case 19:
            return cv::Scalar(124,138,104);
                   
    }
 
}

void cvScalarToGLColor3f(const cv::Scalar& scalar, GLfloat& red, GLfloat& green, GLfloat& blue) {
    red = scalar[0] / 255.0f;
    green = scalar[1] / 255.0f;
    blue = scalar[2] / 255.0f;
}





 
 
 
Eigen::Vector3d SuperEllipsoidCurve(double theta1,double theta2,double p1,double p2)
{
    double ct1,ct2,st1,st2;
    Eigen::Vector3d x;
    double w = 0;

 
    ct1 = cos(theta1);
    ct2 = cos(theta2);
    st1 = sin(theta1);
    st2 = sin(theta2);
  
    x[0] = sign(ct1) * pow(fabs(ct1),p1) * sign(ct2) * pow(fabs(ct2),p2);
    x[2] = sign(ct1) * pow(fabs(ct1),p1) * sign(st2) * pow(fabs(st2),p2);
    x[1] = sign(st1) * pow(fabs(st1),p1);

    // x = R*x;

    return x;
}
 




} //namespace ORB_SLAM

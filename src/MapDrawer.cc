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

//[]
cv::Scalar getColorInmap(int id);
void cvScalarToGLColor3f(const cv::Scalar& scalar, GLfloat& red, GLfloat& green, GLfloat& blue);
// std::vector<Vertex> calculateHyperQuadricVertices(float a, float b, float c, const Eigen::Matrix4f& rotationMatrix);
Eigen::Vector3d SuperEllipsoidCurve(double theta1,double theta2,double p1,double p2);
void FindUnitNormal (double P1[3],double P2[3],double P3[3],double N[3]);




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

//[]
void MapDrawer::DrawMapPoints_background()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;
    //取出所有的地图点
    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    //取出mvpReferenceMapPoints，也即局部地图点
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    //将vpRefMPs从vector容器类型转化为set容器类型，便于使用set::count快速统计 - 我觉得称之为"重新构造"可能更加合适一些
    //补充, set::count用于返回集合中为某个值的元素的个数
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    //显示所有的地图点（不包括局部地图点，对象地图点），大小为2个像素，黑色
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


    //显示局部地图中的背景地图点，大小为2个像素，红色
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
    //取出所有的地图点
    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    //取出mvpReferenceMapPoints，也即局部地图点
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    //将vpRefMPs从vector容器类型转化为set容器类型，便于使用set::count快速统计 - 我觉得称之为"重新构造"可能更加合适一些
    //补充, set::count用于返回集合中为某个值的元素的个数
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    //显示局部地图中的对象地图点，大小为4个像素
    glPointSize(4);
    glBegin(GL_POINTS);
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if((*sit)->whetherInObject == false)
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();

        cv::Scalar color = getColorInmap((*sit)->ObjectId); // 获取对应的颜色
        GLfloat red, green, blue;
        cvScalarToGLColor3f(color, red, green, blue); // 将颜色转换为 glColor3f 格式
        glColor3f(red, green, blue);

        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    //显示所有地图点中的对象地图点（不包括局部地图点），大小为4个像素
    glPointSize(4);
    glBegin(GL_POINTS);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        if(vpMPs[i]->whetherInObject == false)
            continue;

        cv::Scalar color = getColorInmap(vpMPs[i]->ObjectId); // 获取对应的颜色
        GLfloat red, green, blue;
        cvScalarToGLColor3f(color, red, green, blue); // 将颜色转换为 glColor3f 格式
        glColor3f(red, green, blue);

        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

}



//[]绘制地图中的对象质心
void MapDrawer::DrawMapPoints_ObjectCenters()
{
  
    //显示地图中物体质心，大小为8个像素，黄色
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


//[]
void MapDrawer::DrawObjectQuadrics()
{

    for(auto Obj: mpAtlas->GetCurrentMap()->MapObjects->Objects)
    {
        // half axial length.
        float length = Obj.length;
        float width = Obj.width;
        float height = Obj.height;

        cv::Mat axe = cv::Mat::zeros(3,1,CV_32F);
        axe.at<float>(0) = length;
        axe.at<float>(1) = width;
        axe.at<float>(2) = height;

        // quadrcis pose.
        cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
        Twq.at<float>(0, 0) = 1;
        Twq.at<float>(0, 1) = 0;
        Twq.at<float>(0, 2) = 0;
        Twq.at<float>(0, 3) = Obj.Obj_Cen[0];

        Twq.at<float>(1, 0) = 0;
        Twq.at<float>(1, 1) = 1;
        Twq.at<float>(1, 2) = 0;
        Twq.at<float>(1, 3) = Obj.Obj_Cen[1];

        Twq.at<float>(2, 0) = 0;
        Twq.at<float>(2, 1) = 0;
        Twq.at<float>(2, 2) = 1;
        Twq.at<float>(2, 3) = Obj.Obj_Cen[2];

        Twq.at<float>(3, 0) = 0;
        Twq.at<float>(3, 1) = 0;
        Twq.at<float>(3, 2) = 0;
        Twq.at<float>(3, 3) = 1;

        // create a quadric.
        GLUquadricObj *pObj = gluNewQuadric();
        cv::Mat Twq_t = Twq.t();

        // color
        // cv::Scalar sc;
        // sc = cv::Scalar(0, 255, 0);

        cv::Scalar color = getColorInmap(Obj.mnId); // 获取对应的颜色
        GLfloat red, green, blue;
        cvScalarToGLColor3f(color, red, green, blue); // 将颜色转换为 glColor3f 格式
        glColor3f(red, green, blue);


        // add to display list
        glPushMatrix();
        glMultMatrixf(Twq_t.ptr<GLfloat >(0));
        glScalef(
                (GLfloat)(axe.at<float>(0,0)),
                (GLfloat)(axe.at<float>(0,1)),
                (GLfloat)(axe.at<float>(0,2))
                );

        gluQuadricDrawStyle(pObj, GLU_LINE);
        gluQuadricNormals(pObj, GLU_NONE);
        glBegin(GL_COMPILE);
        gluSphere(pObj, 1., 15, 10);

        glEnd();
        glPopMatrix();
        
    }
}


//[]
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
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//将绘制模式设为线框模式
    glEnable(GL_DEPTH_TEST);//启用深度测试和背面剔除
    glEnable(GL_CULL_FACE);
    Eigen::Vector3f ObjCenter = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].Obj_Cen;

    //假定重力方向
    // 步骤1：平移场景，物体质心位于原点(0, 0, 0)
    // glPushMatrix();
    glTranslatef(ObjCenter[0], ObjCenter[1], ObjCenter[2]);
    // 步骤2：定义整体旋转角度
    double rotationX = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].rotationX;
    glRotatef(rotationX, 1.0f, 0.0f, 0.0f);
    glRotatef(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].rotationY, 0.0f, 1.0f, 0.0f);


    // 步骤3：再次平移场景，将点(1, 1, 1)还原回原来的位置
    glTranslatef(-ObjCenter[0], -ObjCenter[1], -ObjCenter[2]);



    cv::Scalar color = getColorInmap(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].mnId); // 获取对应的颜色
    // cv::Scalar color = getColorInmap(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].cid); // 获取对应的颜色
    GLfloat red, green, blue;
    cvScalarToGLColor3f(color, red, green, blue); // 将颜色转换为 glColor3f 格式
    glColor3f(red, green, blue);


    glTranslatef(ObjCenter[0], ObjCenter[1], ObjCenter[2]);//设置位置

    glScalef((GLfloat)(rx), (GLfloat)(ry), (GLfloat)(rz));//设置轴长



    //循环绘制超二次曲面表面
    double dt = 0.01 * TWOPI / res;
    for (int j=0;j<res/2;j++) //纵向划分
    {
        double theta1 = j * TWOPI / (double)res - PID2;
        double theta2 = (j + 1) * TWOPI / (double)res - PID2;

        // 设置线条宽度
        glLineWidth(1.0f);

        glBegin(GL_QUAD_STRIP);

        for (int i=0;i<=res;i++) //横向划分
        {

            if (i == 0 || i == res)
                theta3 = 0;
            else
                theta3 = i * TWOPI / res;


            x = SuperEllipsoidCurve(theta2,theta3,e1,e2);
            // x = Twq*x;
            // glVertex3f(rx*x[0],ry*x[1],rz*x[2]);
            glVertex3f(x[0],x[1],x[2]);


            x = SuperEllipsoidCurve(theta1,theta3,e1,e2);
            // x = Twq*x;
            // glVertex3f(rx*x[0],ry*x[1],rz*x[2]);
            glVertex3f(x[0],x[1],x[2]);


        }
        glEnd();
    }



    glEndList();
    glCallList(superquadric);  //绘制
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);  // 恢复为填充模式

    glPopMatrix();

}

}


void MapDrawer::DrawAnObjectSuperQuadrics(int ID)
{
#define PI 3.14159265358979323846
#define TWOPI 2*3.14159265358979323846
#define PID2 3.14159265358979323846/2.0
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

if(mpAtlas->GetCurrentMap()->MapObjects->Objects.size() == 0)return;

for(int i = 0; i < mpAtlas->GetCurrentMap()->MapObjects->Objects.size(); i++)
{
    if(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].mnId <= ID)continue;
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
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//将绘制模式设为线框模式
    glEnable(GL_DEPTH_TEST);//启用深度测试和背面剔除
    glEnable(GL_CULL_FACE);
    Eigen::Vector3f ObjCenter = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].Obj_Cen;

    //假定重力方向
    // 步骤1：平移场景，物体质心位于原点(0, 0, 0)
    // glPushMatrix();
    glTranslatef(ObjCenter[0], ObjCenter[1], ObjCenter[2]);
    // 步骤2：定义整体旋转角度
    double rotationX = mpAtlas->GetCurrentMap()->MapObjects->Objects[i].rotationX;
    glRotatef(rotationX, 1.0f, 0.0f, 0.0f);
    glRotatef(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].rotationY, 0.0f, 1.0f, 0.0f);


    // 步骤3：再次平移场景，将点(1, 1, 1)还原回原来的位置
    glTranslatef(-ObjCenter[0], -ObjCenter[1], -ObjCenter[2]);



    cv::Scalar color = getColorInmap(mpAtlas->GetCurrentMap()->MapObjects->Objects[i].mnId); // 获取对应的颜色
    GLfloat red, green, blue;
    cvScalarToGLColor3f(color, red, green, blue); // 将颜色转换为 glColor3f 格式
    glColor3f(red, green, blue);


    glTranslatef(ObjCenter[0], ObjCenter[1], ObjCenter[2]);//设置位置

    glScalef((GLfloat)(rx), (GLfloat)(ry), (GLfloat)(rz));//设置轴长



    //循环绘制超二次曲面表面
    double dt = 0.01 * TWOPI / res;
    for (int j=0;j<res/2;j++) //纵向划分
    {
        double theta1 = j * TWOPI / (double)res - PID2;
        double theta2 = (j + 1) * TWOPI / (double)res - PID2;

        // 设置线条宽度
        glLineWidth(1.0f);

        glBegin(GL_QUAD_STRIP);

        for (int i=0;i<=res;i++) //横向划分
        {

            if (i == 0 || i == res)
                theta3 = 0;
            else
                theta3 = i * TWOPI / res;


            x = SuperEllipsoidCurve(theta2,theta3,e1,e2);
            // x = Twq*x;
            // glVertex3f(rx*x[0],ry*x[1],rz*x[2]);
            glVertex3f(x[0],x[1],x[2]);


            x = SuperEllipsoidCurve(theta1,theta3,e1,e2);
            // x = Twq*x;
            // glVertex3f(rx*x[0],ry*x[1],rz*x[2]);
            glVertex3f(x[0],x[1],x[2]);


        }
        glEnd();
    }



    glEndList();
    glCallList(superquadric);  //绘制
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);  // 恢复为填充模式

    glPopMatrix();

}

}








void MapDrawer::DrawMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;
    //取出所有的地图点
    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    //取出mvpReferenceMapPoints，也即局部地图点
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    //将vpRefMPs从vector容器类型转化为set容器类型，便于使用set::count快速统计 - 我觉得称之为"重新构造"可能更加合适一些
    //补充, set::count用于返回集合中为某个值的元素的个数
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    //显示所有的地图点（不包括局部地图点，对象地图点），大小为2个像素，黑色
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


    //显示局部地图中的背景地图点，大小为2个像素，红色
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



    //显示局部地图中的对象地图点，大小为2个像素，绿色
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



    //显示所有地图点中的对象地图点（不包括局部地图点），大小为2个像素，绿色
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
    
    // 使用取余操作循环选择颜色
    int colorCount = 20; // 颜色总数
    int index = id % colorCount; // 计算索引，从0开始

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
    // 将 cv::Scalar 转换为 glColor3f 格式
    red = scalar[0] / 255.0f;
    green = scalar[1] / 255.0f;
    blue = scalar[2] / 255.0f;
}






 
//------ Returns the cross product of 2 vectors
 
void CrossProduct (double M[], double N[], double CS[]) {
    CS[0]=M[1] * N[2] - M[2] * N[1];
    CS[1]=M[2] * N[0] - M[0] * N[2];
    CS[2]=M[0] * N[1] - M[1] * N[0];
}
 
// //------ Returns the length of a vector
 
double GetVectorLength (double M[]) {
    return sqrt( M[0] * M[0] + M[1] * M[1] + M[2] * M[2] );
}
 
// //------ Returns the sum of two vectors
 
void   AddVectors (double M[], double N[], double R[]){
    R[0]= M[0] + N[0];
    R[1]= M[1] + N[1];
    R[2]= M[2] + N[2];
}
 
// //------ Returns the vector scaled by the last parameter
 
void ScaleVector (double M[], double a, double N[]) {
    N[0]= M[0] * a;
    N[1]= M[1] * a; 
    N[2]= M[2] * a;
}
 
//------ Returns a normalized vector (length = 1)
 
void NormalizeVector (double M[],double R[3]) {
    double norm = GetVectorLength(M);
    if (norm == 0) norm  =1.0;
    ScaleVector( M, 1./ norm, R );
}
 
//------ Returns the unit normal vector of a triangle specified by the three
// points P1, P2, and P3.
 
void FindUnitNormal (double P1[3],double P2[3],double P3[3],double N[3]){
    double D1[3],D2[3];
    double R[3];
    D1[0]=P1[0]-P2[0];
    D1[1]=P1[1]-P2[1];
    D1[2]=P1[2]-P2[2];
    D2[0]=P2[0]-P3[0];
    D2[1]=P2[1]-P3[1];
    D2[2]=P2[2]-P3[2];
    CrossProduct(D1,D2,R);
    NormalizeVector(R,N);
}
 
 
 
Eigen::Vector3d SuperEllipsoidCurve(double theta1,double theta2,double p1,double p2)
{
    double ct1,ct2,st1,st2;
    Eigen::Vector3d x;
    double w = 0;

    // // 将角度转换为弧度
    // double rotationAngleY = 45.0 * M_PI / 180.0;
    // double rotationAngleX = 20.0 * M_PI / 180.0;
    // Eigen::Matrix3d R;
    // R << 1, 0, 0,
    //  0, cos(rotationAngleX), -sin(rotationAngleX),
    //  0, sin(rotationAngleX), cos(rotationAngleX);
 
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
 

 
// void reshape (int w, int h)
// {
//     if (h == 0 || w == 0) return;  
 
//     glViewport( 0, 0, w, h );
//     glMatrixMode(GL_PROJECTION);
//     glLoadIdentity();
//     gluPerspective( 60, h ? w / h : 0, 1, 20 );
//     glMatrixMode(GL_MODELVIEW);
//     glLoadIdentity();
// }
 



} //namespace ORB_SLAM

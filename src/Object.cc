/**

* Version: 1.0
* Created: 2023/05/06
* Author: Shize Wang

*/

#include "Object.h"

namespace ORB_SLAM3
{

bool areLinesParallel(const cv::Vec4f& line1, const cv::Vec4f& line2);
double calculateSlope(const cv::Vec4f& line);
double CalculateAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& q1, const cv::Point& q2);


//[]使用帧初始化对象
Object Object::InitialObject(Frame currentFrame, YOLO currentYolo)
{
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    ///当前帧中的特征点是否在地图中的标记
    ///当前帧的特征点在地图中是否出现;后者是表示地图中没有出现,但是在当前帧中是第一次被观测得到的点    
    vector<bool> mvbMap, mvbVO;

    vector<int>cidVec;
    vector<vector<cv::KeyPoint>>Obj_KPsVec(currentYolo.yoloLines.size());
    vector<vector<MapPoint*>>Obj_MPsVec(currentYolo.yoloLines.size());

    vCurrentKeys = currentFrame.mvKeysUn;
    
    int N = vCurrentKeys.size();
    cout<<"Keypoint size:"<< N <<endl;

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    //判断当前特征点是否有对应的地图点
    //获取当前帧地图点的信息
    for(int i = 0; i < N; i++)
    {
        MapPoint* pMP = currentFrame.mvpMapPoints[i];
        if(pMP)
        {
            if(!currentFrame.mvbOutlier[i])
            {
                //该mappoints可以被多帧观测到，则为有效的地图点
                if(pMP->Observations()>0)
                    mvbMap[i]=true;
                else
                //否则表示这个特征点是在当前帧中第一次提取得到的点
                    mvbVO[i]=true;
            }
        }
    }

    //判断特征点是否在对象上
    for(int i = 0; i < N; i++)
    {
        //1.如果这个点在视觉里程计中有,在局部地图中也有
        if(mvbVO[i] || mvbMap[i])
        {
            for(int j = 0; j < currentYolo.yoloLines.size(); j++)
            {   
                if(currentYolo.yoloLines[j].mask.size() <= 4)continue;
                // if(stoi(currentYolo.yoloLines[j].cla) == 60 ||
                //    stoi(currentYolo.yoloLines[j].cla) == 72 ||
                //    stoi(currentYolo.yoloLines[j].cla) == 32 ||
                //    stoi(currentYolo.yoloLines[j].cla) == 0 || 
                //    stoi(currentYolo.yoloLines[j].cla) == 15 ||
                //    stoi(currentYolo.yoloLines[j].cla) == 63 ||
                //    stoi(currentYolo.yoloLines[j].cla) == 68 ||
                //    stoi(currentYolo.yoloLines[j].cla) == 58)continue;//删掉桌子、植物、冰箱、体育球、人、猫、笔记本电脑、微波炉
                if(
                   stoi(currentYolo.yoloLines[j].cla) == 60
                  )continue;
                if(currentYolo.yoloLines[j].cT < 0.5)continue;//删掉置信度小于0.5的
                // if(!((stoi(currentYolo.yoloLines[j].cla) == 75) || (stoi(currentYolo.yoloLines[j].cla) == 58)))continue;
                // if(!(stoi(currentYolo.yoloLines[j].cla) == 56))continue;

                //2.先判断点是否在box里
                double box_leftx = currentYolo.yoloLines[j].box.x - currentYolo.yoloLines[j].box.width/2;
                double box_rightx = currentYolo.yoloLines[j].box.x + currentYolo.yoloLines[j].box.width/2;
                double box_lefty = currentYolo.yoloLines[j].box.y - currentYolo.yoloLines[j].box.height/2;
                double box_righty = currentYolo.yoloLines[j].box.y + currentYolo.yoloLines[j].box.height/2;

                if(box_leftx < 5 || box_lefty < 5 || box_rightx > 675 || box_righty > 475)continue;

                if(vCurrentKeys[i].pt.x >= box_leftx && vCurrentKeys[i].pt.x <= box_rightx)
                {
                    if(vCurrentKeys[i].pt.y >= box_lefty && vCurrentKeys[i].pt.y <= box_righty)
                    {
                        //3.后判断点是否在mask里，opencv自带函数，这个效果好一点
                        cv::Point point(vCurrentKeys[i].pt.x, vCurrentKeys[i].pt.y);                   
                        /*cv::pointPolygonTest
                        执行点对轮廓的测试。该函数确定点是在轮廓内部、外部还是位于边缘（或与顶点重合）。
                        它分别返回正值（在内部）、负值（在外部）或零值（在边缘上）。
                        当`measureDist`为`false`时，返回值为+1（内部）、-1（外部）或0（在边缘上）。
                        否则，返回值是点与最近轮廓边缘之间的有符号距离。下面是函数的一个示例输出，其中每个图像像素与轮廓进行了测试。

                        参数:
                        contour - 输入的轮廓。
                        pt - 要对轮廓进行测试的点。
                        measureDist - 如果为true，函数估计点到最近轮廓边缘的有符号距离。否则，函数仅检查点是否在轮廓内部。*/
                        if(cv::pointPolygonTest(currentYolo.yoloLines[j].mask, point, false) >= 0)
                        {   

                            //特征点集
                            Obj_KPsVec[j].push_back(vCurrentKeys[i]);
                            //修改地图点对象判断符
                            currentFrame.mvpMapPoints[i]->whetherInObject = true;
                            //地图点集
                            Obj_MPsVec[j].push_back(currentFrame.mvpMapPoints[i]);
                            // cout<<"object "<<j<<"mappoint pose : "<<currentFrame.mvpMapPoints[i]->GetWorldPos()<<endl;
                        }
                       
                    }
                }
                            
            }
        }
    }
    

    for(int j = 0; j < currentYolo.yoloLines.size(); j++)
    {
        cidVec.push_back(stoi(currentYolo.yoloLines[j].cla));
    }

    cout<<"Obj_KPsVec size:"<<Obj_KPsVec.size()<<endl;


    Object curfObject(cidVec, Obj_KPsVec, Obj_MPsVec);

    for(int k = 0; k < curfObject.Objects.size(); k++)
    {
    curfObject.Objects[k].Obj_Fms.push_back(currentFrame.mnId);
    curfObject.Objects[k].FrameId = currentFrame.mnId;
    curfObject.Objects[k].FramePose = currentFrame.GetCameraCenter();
    curfObject.Objects[k].Obj_box = cv::Rect(currentYolo.yoloLines[k].box.x - currentYolo.yoloLines[k].box.width/2, currentYolo.yoloLines[k].box.y - currentYolo.yoloLines[k].box.height/2, 
                                             currentYolo.yoloLines[k].box.width, currentYolo.yoloLines[k].box.height);

    // curfObject.Objects[k].Obj_box = cv::Rect(currentYolo.yoloLines[k].box.x/2 - currentYolo.yoloLines[k].box.width/2, currentYolo.yoloLines[k].box.y/2 - currentYolo.yoloLines[k].box.height/2, 
    //                                          currentYolo.yoloLines[k].box.width, currentYolo.yoloLines[k].box.height);
    
    curfObject.Objects[k].mask = currentYolo.yoloLines[k].mask;
    curfObject.Objects[k].WhetherInFrame = true;
    curfObject.Objects[k].Score = currentYolo.yoloLines[k].cT;
    // ComputeRotation(curfObject.Objects[k], currentFrame);
    if(curfObject.Objects[k].cid == 13 || curfObject.Objects[k].cid == 26 || curfObject.Objects[k].cid == 28 || curfObject.Objects[k].cid == 30 
    || curfObject.Objects[k].cid == 31 || curfObject.Objects[k].cid == 33 || curfObject.Objects[k].cid == 36 || curfObject.Objects[k].cid == 37 
    || curfObject.Objects[k].cid == 56 || curfObject.Objects[k].cid == 57 || curfObject.Objects[k].cid == 59 || curfObject.Objects[k].cid == 60 
    || curfObject.Objects[k].cid == 62 || curfObject.Objects[k].cid == 63 || curfObject.Objects[k].cid == 64 || curfObject.Objects[k].cid == 65
    || curfObject.Objects[k].cid == 66 || curfObject.Objects[k].cid == 67 || curfObject.Objects[k].cid == 68 || curfObject.Objects[k].cid == 69
    || curfObject.Objects[k].cid == 70 || curfObject.Objects[k].cid == 71 || curfObject.Objects[k].cid == 72 || curfObject.Objects[k].cid == 73)
    // if(curfObject.Objects[k].cid == 13)
    {
        curfObject.Objects[k].e1 = 0.1;  
        curfObject.Objects[k].e2 = 0.1;  
    }
    else if(curfObject.Objects[k].cid == 10 || curfObject.Objects[k].cid == 39|| curfObject.Objects[k].cid == 40 || curfObject.Objects[k].cid == 41
    || curfObject.Objects[k].cid == 45 || curfObject.Objects[k].cid == 54 || curfObject.Objects[k].cid == 55 || curfObject.Objects[k].cid == 75)
    // else if(curfObject.Objects[k].cid == 10 || curfObject.Objects[k].cid == 39|| curfObject.Objects[k].cid == 40 || curfObject.Objects[k].cid == 41
    // || curfObject.Objects[k].cid == 54 || curfObject.Objects[k].cid == 55 || curfObject.Objects[k].cid == 0)
    {
        curfObject.Objects[k].e1 = 0.1;  
        curfObject.Objects[k].e2 = 1;  
    }
    else
    {
        curfObject.Objects[k].e1 = 1;  
        curfObject.Objects[k].e2 = 1; 
    }
    }


    //删除坏物体
    // 创建一个用于存储要删除的元素位置的向量
    std::vector<int> indicesToDelete;
    // 遍历向量并记录要删除的元素位置
    for (int k = 0; k < curfObject.Objects.size(); k++)
    {   
        curfObject.Objects[k].WhetherInFrame = false;

        int MPsize = curfObject.Objects[k].Obj_MPs.size();

        if (MPsize <= 10) //地图点少于10的
        {
            indicesToDelete.push_back(k);
            for(int l = 0; l < MPsize; l++)
            {
                curfObject.Objects[k].Obj_MPs[l]->whetherInObject = false;
            }
        }
    }
    // 根据记录的位置删除元素（从后往前删除）
    for (int i = indicesToDelete.size() - 1; i >= 0; i--)
    {   
        if(indicesToDelete.empty())continue;
        int index = indicesToDelete[i];
        curfObject.Objects.erase(curfObject.Objects.begin() + index);
    }

    //删除IoU过大的物体
    indicesToDelete.clear();
    for (int k = 0; k < curfObject.Objects.size(); k++)
    {   
        for (int j = k; j < curfObject.Objects.size(); j++)
        {
            if(j+1 >= curfObject.Objects.size())continue;
            else
            {
                curfObject.Objects[k].WhetherInFrame = false;

                int MPsize = curfObject.Objects[k].Obj_MPs.size();

                //计算IoU
                cv::Rect rectI = curfObject.Objects[k].Obj_box & curfObject.Objects[j+1].Obj_box;
                cv::Rect rectU = curfObject.Objects[k].Obj_box | curfObject.Objects[j+1].Obj_box;
                double IoUrate2D = rectI.area() *1.0/ rectU.area();

                cout<<"IoU in initial: "<<IoUrate2D<<" "<<curfObject.Objects.size()<<" "<<k<<" "<<j+1<<endl;
                if(IoUrate2D > 0.5)
                {
                    indicesToDelete.push_back(k);
                    for(int l = 0; l < MPsize; l++)
                    {
                        curfObject.Objects[k].Obj_MPs[l]->whetherInObject = false;
                    }
                }
            }
        }
    }

    for (int i = indicesToDelete.size() - 1; i >= 0; i--)
    {   
        if(indicesToDelete.empty())continue;
        int index = indicesToDelete[i];
        curfObject.Objects.erase(curfObject.Objects.begin() + index);
    }


    curfObject.printObject();

    // //剔除外点 防止xyz初值过大
    // for(int i = 0; i < curfObject.Objects.size(); i++)
    // Object::RemoveOutlierD(curfObject.Objects[i], 1);



    //初始化长宽高
    for(int i = 0; i < curfObject.Objects.size(); i++)
    {
        int goodNumber = 0;

        if(curfObject.Objects[i].Obj_MPs.size() <= 2)continue;
        else
        {
            Eigen::Vector3f sum_pos_3d(0.0f, 0.0f, 0.0f);
            MapPoint * pMP;
            float length = 0;
            float width = 0;
            float height = 0;
            for (int j = 0; j < curfObject.Objects[i].Obj_MPs.size(); j++)
            {
                pMP = curfObject.Objects[i].Obj_MPs[j];
                Eigen::Vector3f pos = pMP->GetWorldPos();
                if (pMP->isBad())continue;                
                sum_pos_3d += pos;
                goodNumber ++;
            }
            // mean(3d center)
            curfObject.Objects[i].Obj_Cen = sum_pos_3d / goodNumber;

            //计算长宽高
            for (int j = 0; j < curfObject.Objects[i].Obj_MPs.size(); j++)
            {
                pMP = curfObject.Objects[i].Obj_MPs[j];
                Eigen::Vector3f pos = pMP->GetWorldPos();
                if (pMP->isBad())continue; 
                if(abs(pos.x() - curfObject.Objects[i].Obj_Cen.x()) >= length)  
                    length = abs(pos.x() - curfObject.Objects[i].Obj_Cen.x());
                if(abs(pos.y() - curfObject.Objects[i].Obj_Cen.y()) >= width)  
                    width = abs(pos.y() - curfObject.Objects[i].Obj_Cen.y());
                if(abs(pos.z() - curfObject.Objects[i].Obj_Cen.z()) >= height)  
                    height = abs(pos.z() - curfObject.Objects[i].Obj_Cen.z());
            }
            curfObject.Objects[i].length  = length;
            curfObject.Objects[i].width  = width;
            curfObject.Objects[i].height  = height;



            //计算旋转矩阵，旋转重力方向设定角度
            cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
            float RotationX = curfObject.Objects[i].rotationX * M_PI / 180.0f;
            Twq.at<float>(0, 0) = 1;
            Twq.at<float>(0, 1) = 0;
            Twq.at<float>(0, 2) = 0;
            Twq.at<float>(0, 3) = curfObject.Objects[i].Obj_Cen[0];

            Twq.at<float>(1, 0) = 0;
            Twq.at<float>(1, 1) = cos(RotationX);
            Twq.at<float>(1, 2) = -sin(RotationX);
            Twq.at<float>(1, 3) = curfObject.Objects[i].Obj_Cen[1];

            Twq.at<float>(2, 0) = 0;
            Twq.at<float>(2, 1) = sin(RotationX);
            Twq.at<float>(2, 2) = cos(RotationX);
            Twq.at<float>(2, 3) = curfObject.Objects[i].Obj_Cen[2];
            
            Twq.at<float>(3, 0) = 0;
            Twq.at<float>(3, 1) = 0;
            Twq.at<float>(3, 2) = 0;
            Twq.at<float>(3, 3) = 1;
            

            curfObject.Objects[i].Twq = Twq;

        }
    }

    // curfObject.ComputeObjectCenter();


   

    return curfObject;

}



//作用：判断点是否在多边形内 没用
//p指目标点， Polygon指多边形的点集合
bool Object::InPolygon (cv::Point p, vector<cv::Point>& Polygon)  
{  

	// 交点个数  
	int nCross = 0;  
	for (int i = 0; i < Polygon.size(); i++)   
	{  
		cv::Point p1 = Polygon[i];  
		cv::Point p2 = Polygon[(i + 1) % Polygon.size()];// 点P1与P2形成连线  
 
		if ( p1.y == p2.y )  
			continue;  
		if ( p.y < min(p1.y, p2.y) )  
			continue;  
		if ( p.y >= max(p1.y, p2.y) )  
			continue;  
		// 求交点的x坐标（由直线两点式方程转化而来）   
 
		double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;  
 
		// 只统计p1p2与p向右射线的交点  
		if ( x > p.x )  
		{  
			nCross++;  
		}  
 
	}  
 
	// 交点为偶数，点在多边形之外  
	// 交点为奇数，点在多边形之内
	if ((nCross % 2) == 1)
	{
		// cout<<"点在区域内"<<endl;
		return true;
	}
	else
	{
		// cout<<"点在区域外"<<endl;
		return false;
	}
} 

//计算物体质心
void Object::ComputeObjectCenter()
{
//目前常见的有两种方法，一种计算最大最小点取平均，一种直接所有点取平均。

//所有点取平均的方法
    for(int i = 0; i < this->Objects.size(); i++)
    {
        int goodNumber = 0;

        if(this->Objects[i].Obj_MPs.size() <= 2)continue;
        else
        {
            Eigen::Vector3f sum_pos_3d(0.0f, 0.0f, 0.0f);
            MapPoint * pMP;
            float length = 0;
            float width = 0;
            float height = 0;
            for (int j = 0; j < this->Objects[i].Obj_MPs.size(); j++)
            {
                pMP = Objects[i].Obj_MPs[j];
                Eigen::Vector3f pos = pMP->GetWorldPos();
                if (pMP->isBad())continue;                
                sum_pos_3d += pos;
                goodNumber ++;
            }
            // mean(3d center)
            this->Objects[i].Obj_Cen = sum_pos_3d / goodNumber;

            /*跑长方体demo时需要更改质心计算方式
            // float max_x = 0;
            // float min_x = 100;
            // float max_y = 0;
            // float min_y = 100;
            // float max_z = 0;
            // float min_z = 100;
  
            // for (int j = 0; j < this->Objects[i].Obj_MPs.size(); j++)
            // {
            //     pMP = Objects[i].Obj_MPs[j];
            //     Eigen::Vector3f pos = pMP->GetWorldPos();
            //     if (pMP->isBad())continue;  
            //     if(pos.x() > max_x)max_x = pos.x();          
            //     if(pos.x() < min_x)min_x = pos.x();          
            //     if(pos.y() > max_y)max_y = pos.y();          
            //     if(pos.y() < min_y)min_y = pos.y();          
            //     if(pos.z() > max_z)max_z = pos.z();          
            //     if(pos.z() < min_z)min_z = pos.z();          
            //     sum_pos_3d += pos;
            //     goodNumber ++;
            // }
            // this->Objects[i].Obj_Cen[0] = (max_x + min_x)/2;
            // this->Objects[i].Obj_Cen[1] = (max_y + min_y)/2;
            // this->Objects[i].Obj_Cen[2] = (max_z + min_z)/2;
            */


            // //计算长宽高
            // for (int j = 0; j < this->Objects[i].Obj_MPs.size(); j++)
            // {
            //     pMP = Objects[i].Obj_MPs[j];
            //     Eigen::Vector3f pos = pMP->GetWorldPos();
            //     if (pMP->isBad())continue; 
            //     // if(abs(pos.x() - this->Objects[i].Obj_Cen.x()) >= length)  
            //         length += abs(pos.x() - this->Objects[i].Obj_Cen.x());
            //     // if(abs(pos.y() - this->Objects[i].Obj_Cen.y()) >= width)  
            //         width += abs(pos.y() - this->Objects[i].Obj_Cen.y());
            //     // if(abs(pos.z() - this->Objects[i].Obj_Cen.z()) >= height)  
            //         height += abs(pos.z() - this->Objects[i].Obj_Cen.z());
            // }
            // this->Objects[i].length  = length/ goodNumber;
            // this->Objects[i].width  = width/ goodNumber;
            // this->Objects[i].height  = height/ goodNumber;


            // //重力方向
            // float angleInRadians = this->Objects[i].rotationX * M_PI / 180.0f;
            // Eigen::Matrix3f R_Eigen;
            // R_Eigen << 1, 0, 0,
            //                 0, cos(angleInRadians), -sin(angleInRadians),
            //                 0, sin(angleInRadians), cos(angleInRadians);

            //计算长宽高
            for (int j = 0; j < this->Objects[i].Obj_MPs.size(); j++)
            {
                pMP = Objects[i].Obj_MPs[j];
                Eigen::Vector3f pos = pMP->GetWorldPos();
                if(pMP->isBad())continue; 
                if(abs(pos.x() - this->Objects[i].Obj_Cen.x()) >= length)  
                    length = abs(pos.x() - this->Objects[i].Obj_Cen.x());
                if(abs(pos.y() - this->Objects[i].Obj_Cen.y()) >= width)  
                    width = abs(pos.y() - this->Objects[i].Obj_Cen.y());
                if(abs(pos.z() - this->Objects[i].Obj_Cen.z()) >= height)  
                    height = abs(pos.z() - this->Objects[i].Obj_Cen.z());
            }
            if(this->Objects[i].length  > length)this->Objects[i].length  = length;
            if(this->Objects[i].width  > width)this->Objects[i].width  = width;
            if(this->Objects[i].height  > height)this->Objects[i].height  = height;



            //计算旋转矩阵，旋转重力方向设定角度
            cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
            float RotationX = Objects[i].rotationX * M_PI / 180.0f;
            float RotationY = Objects[i].rotationY * M_PI / 180.0f;

            Twq.at<float>(0, 0) = 1;
            Twq.at<float>(0, 1) = 0;
            Twq.at<float>(0, 2) = 0;
            Twq.at<float>(0, 3) = this->Objects[i].Obj_Cen[0];

            Twq.at<float>(1, 0) = 0;
            Twq.at<float>(1, 1) = cos(RotationX);
            Twq.at<float>(1, 2) = -sin(RotationX);
            Twq.at<float>(1, 3) = this->Objects[i].Obj_Cen[1];

            Twq.at<float>(2, 0) = 0;
            Twq.at<float>(2, 1) = sin(RotationX);
            Twq.at<float>(2, 2) = cos(RotationX);
            Twq.at<float>(2, 3) = this->Objects[i].Obj_Cen[2];
            
            Twq.at<float>(3, 0) = 0;
            Twq.at<float>(3, 1) = 0;
            Twq.at<float>(3, 2) = 0;
            Twq.at<float>(3, 3) = 1;

            cv::Mat Twq_R = cv::Mat::zeros(4,4,CV_32F);

            Twq_R.at<float>(0, 0) = cos(RotationY);
            Twq_R.at<float>(0, 1) = 0;
            Twq_R.at<float>(0, 2) = sin(RotationY);
            Twq_R.at<float>(0, 3) = 0;

            Twq_R.at<float>(1, 0) = 0;
            Twq_R.at<float>(1, 1) = 1;
            Twq_R.at<float>(1, 2) = 0;
            Twq_R.at<float>(1, 3) = 0;

            Twq_R.at<float>(2, 0) = -sin(RotationY);
            Twq_R.at<float>(2, 1) = 0;
            Twq_R.at<float>(2, 2) = cos(RotationY);
            Twq_R.at<float>(2, 3) = 0;
            
            Twq_R.at<float>(3, 0) = 0;
            Twq_R.at<float>(3, 1) = 0;
            Twq_R.at<float>(3, 2) = 0;
            Twq_R.at<float>(3, 3) = 1;

            

            this->Objects[i].Twq = Twq * Twq_R;

        }
    }

}






//没用
void Object::Association(Object currentObjects, Object& AllObjects, Object& newobjects)
{
    currentObjects.ComputeObjectCenter();//算一下新建的物体质心

    //1.TODO:使用地图点关联物体,使用set查找，可以优化
    // 定义比较函数
    struct MapPointComparator 
    {
        bool operator()(const MapPoint* mp1, const MapPoint* mp2) const 
        {
            return mp1->mnId < mp2->mnId;
        }
    };
    //判断是否已经关联过
    vector<bool> associated(currentObjects.Objects.size(), false);

    for(int i = 0; i < AllObjects.Objects.size(); i++)//所有地图中的物体
    {       
        // cout<<"Objects_map size:"<<AllObjects.Objects.size()<<endl;
        for(int j = 0; j < currentObjects.Objects.size(); j++)//所有当前帧的物体
        {
            if(currentObjects.Objects[j].Obj_MPs.size() < 5)continue;
            int samepoints = 0;
            set<MapPoint*, MapPointComparator> AllMappoints(AllObjects.Objects[i].Obj_MPs.begin(), AllObjects.Objects[i].Obj_MPs.end());
            set<MapPoint*, MapPointComparator> CurrentMappoints(currentObjects.Objects[j].Obj_MPs.begin(), currentObjects.Objects[j].Obj_MPs.end());

            for (const auto& mp : CurrentMappoints) 
            {
                if (AllMappoints.count(mp) > 0) 
                {
                    cout << "Found MapPoint with mnId " << mp->mnId << endl;
                    samepoints++;
                }
            }

            if(samepoints >= 10 && associated[j] == false)//相同地图点大于等于10个，就视为一个物体
            {   
                // cout<<"before associate: "<<AllObjects.Objects[i].Obj_MPs.size();
                AssociateObject(currentObjects.Objects[j], AllObjects.Objects[i]);
                associated[j] = true;
                // cout<<"after associate: "<<AllObjects.Objects[i].Obj_MPs.size();
                double MPrate = static_cast<double>(samepoints)/currentObjects.Objects[j].Obj_MPs.size();
                // cout<<"rate of mappoints: "<<MPrate<<endl;
                cout<<"associate use mappoint: "<<j<<endl;
                continue;
            }
            AllMappoints.clear();
            CurrentMappoints.clear();


            //2.使用质心距离和类别来关联
            float CenterDistance = (currentObjects.Objects[j].Obj_Cen - AllObjects.Objects[i].Obj_Cen).norm();
            // cout<<"distance :"<<CenterDistance<<endl;

            double CLrate = 0;
            if(currentObjects.Objects[j].cid == AllObjects.Objects[i].cid)
            CLrate += 0.5;
            if(CenterDistance < 0.5)
            CLrate += 0.4*(1-(2*CenterDistance));
            if(samepoints>=0)
            CLrate += 0.1*static_cast<double>(samepoints)/currentObjects.Objects[j].Obj_MPs.size();
            // cout<<"rate of class :"<<CLrate<<endl;


            if(currentObjects.Objects[j].cid == AllObjects.Objects[i].cid && CenterDistance < 0.4 && associated[j] == false  && samepoints >= 3)
            // if(currentObjects.Objects[j].cid == AllObjects.Objects[i].cid && CenterDistance < 0.4 && associated[j] == false )
            {
                AssociateObject(currentObjects.Objects[j], AllObjects.Objects[i]);
                associated[j] = true;
                cout<<"associate use class: "<<j<<endl;
                continue;
            }

            //3.使用IoU来关联
            cv::Rect rectI = AllObjects.Objects[i].Obj_box & currentObjects.Objects[j].Obj_box;
            cv::Rect rectU = AllObjects.Objects[i].Obj_box | currentObjects.Objects[j].Obj_box;
            // cout<<"map obj area: "<<AllObjects.Objects[i].Obj_box.area()<<endl;
            // cout<<"cur obj area: "<<currentObjects.Objects[j].Obj_box.area()<<endl;
            // cout<<"rectI: "<<rectI<<endl;
            // cout<<"rectU: "<<rectU<<endl;
            double IoU = rectI.area() *1.0/ rectU.area();
            // cout<<"rete of IoU :"<<IoU<<endl;
            if(IoU > 0.5 && associated[j] == false && samepoints >= 3)
            {
                AssociateObject(currentObjects.Objects[j], AllObjects.Objects[i]);
                associated[j] = true;
                cout<<"associate use IoU: "<<j<<endl;
            }

            //4.判断地图点是否在物体内来关联
            if(associated[j] == false && Calculate3DIoU(currentObjects.Objects[j], AllObjects.Objects[i]) >= 0.1)
            {
                if(WhetherInSuperquadric(currentObjects.Objects[j], AllObjects.Objects[i]) > 0)
                {
                    AssociateObject(currentObjects.Objects[j], AllObjects.Objects[i]);
                    associated[j] = true;
                    cout<<"associate use Superquadric: "<<j<<endl;
                }
            }



        }
    }

  

    //5.添加新物体
    for(int i = 0; i < currentObjects.Objects.size(); i++)
    {
        cout<<i<<" associated is "<<associated[i]<<endl;
        if(associated[i] == true)continue;
        if(currentObjects.Objects[i].Obj_MPs.size() <= 5)continue;//地图点少于5的物体不要
        newobjects.Objects.push_back(currentObjects.Objects[i]);
        cout<<"newobjects in object"<<newobjects.Objects.size()<<endl;
    }
            
    // return newobjects;

}



//合并物体
void Object::AssociateObject(Object::Object_3D currentObject, Object::Object_3D& MapObject)
{

    struct MapPointComparator 
    {
        bool operator()(const MapPoint* mp1, const MapPoint* mp2) const 
        {
            return mp1->mnId < mp2->mnId;
        }
    };
    vector<MapPoint *>difMappoints;
    set<MapPoint*, MapPointComparator> AllMappoints(MapObject.Obj_MPs.begin(), MapObject.Obj_MPs.end());
    set<MapPoint*, MapPointComparator> CurrentMappoints(currentObject.Obj_MPs.begin(), currentObject.Obj_MPs.end());


    for (const auto& mp : CurrentMappoints) 
    {
        if (AllMappoints.count(mp) == 0) 
        {
            if(mp->isBad())continue;
            difMappoints.push_back(mp);
        }
    }

    //添加不同地图点
    for(int m = 0; m < difMappoints.size(); m++)
    {
        //修改地图点id
        difMappoints[m]->ObjectId = MapObject.mnId;
        MapObject.Obj_MPs.push_back(difMappoints[m]);
    }

    //添加帧信息
    // if(MapObject.Obj_Fms.back() != currentObject.Obj_Fms.back())
    MapObject.Obj_Fms.push_back(currentObject.Obj_Fms.back());
    MapObject.FramePose = currentObject.FramePose;

    //添加类别信息
    MapObject.cid = currentObject.cid;


    //添加mask box信息
    MapObject.Obj_box = currentObject.Obj_box;
    MapObject.mask = currentObject.mask;

    // MapObject.rotationY = currentObject.rotationY;


    difMappoints.clear();
    AllMappoints.clear();
    CurrentMappoints.clear();

}


//计算均值和标准差
void Object::CommputeDistanceSD(Object::Object_3D& Object)
{
    Eigen::VectorXf distance(Object.Obj_MPs.size());
    for(int i = 0; i < Object.Obj_MPs.size(); i++)
    {
        distance(i) = (Object.Obj_Cen - Object.Obj_MPs[i]->GetWorldPos()).norm();
    }

    Object.mean = distance.mean();
    Object.SD = sqrt((distance.array() - Object.mean).square().mean());
}

//计算xyz均值和标准差
void Object::CommputeDistanceSDXYZ(Object::Object_3D& Object)
{
    Eigen::VectorXf distanceX(Object.Obj_MPs.size());
    Eigen::VectorXf distanceY(Object.Obj_MPs.size());
    Eigen::VectorXf distanceZ(Object.Obj_MPs.size());
    for(int i = 0; i < Object.Obj_MPs.size(); i++)
    {
        distanceX(i) = abs(Object.Obj_Cen.x() - Object.Obj_MPs[i]->GetWorldPos().x());
        distanceY(i) = abs(Object.Obj_Cen.y() - Object.Obj_MPs[i]->GetWorldPos().y());
        distanceZ(i) = abs(Object.Obj_Cen.z() - Object.Obj_MPs[i]->GetWorldPos().z());
    }

    Object.meanX = distanceX.mean();
    Object.SDX = sqrt((distanceX.array() - Object.meanX).square().mean());
    Object.meanY = distanceY.mean();
    Object.SDY = sqrt((distanceY.array() - Object.meanY).square().mean());
    Object.meanZ = distanceZ.mean();
    Object.SDZ = sqrt((distanceZ.array() - Object.meanZ).square().mean());
}

//计算depth均值和标准差
void Object::CommputeDDistanceSD(Object::Object_3D& Object)
{
    Eigen::VectorXf distance(Object.Obj_MPs.size());
    for(int i = 0; i < Object.Obj_MPs.size(); i++)
    {
        distance(i) = abs((Object.Obj_Cen - Object.FramePose).norm() - (Object.Obj_MPs[i]->GetWorldPos() - Object.FramePose).norm());
        // distance(i) = abs((Object.Obj_MPs[i]->GetWorldPos() - Object.FramePose).norm());
    }

    Object.mean = distance.mean();
    Object.SD = sqrt((distance.array() - Object.mean).square().mean());
}

//剔除外点
void Object::RemoveOutlier(Object &Objs)
{
    //1.正态分布剔除2个标准差外的点

    // 创建一个用于存储要删除的元素位置的向量
    std::vector<int> MpToDelete;

    for(int k = 0; k < Objs.Objects.size(); k++)
    {
        if(Objs.Objects[k].Obj_MPs.size() < 30)continue;//中心极限定理
        CommputeDistanceSD(Objs.Objects[k]);
        // 遍历向量并记录要删除的元素位置
        for (int i = 0; i < Objs.Objects[k].Obj_MPs.size(); i++)
        {
            // cout<<"compute distance: "<<(obj.Obj_Cen - obj.Obj_MPs[i]->GetWorldPos()).norm()<<endl;
            // cout<<"mean distance: "<<(obj.mean + 1 * obj.SD)<<endl;
            if ((Objs.Objects[k].Obj_Cen - Objs.Objects[k].Obj_MPs[i]->GetWorldPos()).norm() > (Objs.Objects[k].mean + 3 * Objs.Objects[k].SD))
            {
                MpToDelete.push_back(i);
                Objs.Objects[k].Obj_MPs[i]->whetherInObject = false;
            }
        }

        // 根据记录的位置删除元素（从后往前删除）
        for (int i = MpToDelete.size() - 1; i >= 0; i--)
        {   
            if(MpToDelete.empty())continue;
            int index = MpToDelete[i];
            Objs.Objects[k].Obj_MPs.erase(Objs.Objects[k].Obj_MPs.begin() + index);
        }
        MpToDelete.clear();

    }

    
}


//剔除外点
void Object::RemoveOutlierXYZ(Object &Objs)
{
    //1.正态分布剔除3个标准差外的点


    for(int k = 0; k < Objs.Objects.size(); k++)
    {

        if(Objs.Objects[k].Obj_MPs.size() < 30)continue;

        // 创建一个用于存储要删除的元素位置的向量
        std::vector<int> MpToDelete(Objs.Objects[k].Obj_MPs.size(), 0);

        CommputeDistanceSDXYZ(Objs.Objects[k]);
        // 遍历向量并记录要删除的元素位置
        for (int i = 0; i < Objs.Objects[k].Obj_MPs.size(); i++)
        {
            if (abs(Objs.Objects[k].Obj_Cen.x() - Objs.Objects[k].Obj_MPs[i]->GetWorldPos().x()) > (Objs.Objects[k].meanX + 3 * Objs.Objects[k].SDX))
            {
                MpToDelete[i] = 1;
                Objs.Objects[k].Obj_MPs[i]->whetherInObject = false;
            }
            if (abs(Objs.Objects[k].Obj_Cen.y() - Objs.Objects[k].Obj_MPs[i]->GetWorldPos().y()) > (Objs.Objects[k].meanY + 3 * Objs.Objects[k].SDY))
            {
                MpToDelete[i] = 1;
                Objs.Objects[k].Obj_MPs[i]->whetherInObject = false;
            }
            if (abs(Objs.Objects[k].Obj_Cen.z() - Objs.Objects[k].Obj_MPs[i]->GetWorldPos().z()) > (Objs.Objects[k].meanZ + 3 * Objs.Objects[k].SDZ))
            {
                MpToDelete[i] = 1;
                Objs.Objects[k].Obj_MPs[i]->whetherInObject = false;
            }
        }



        // 根据记录的位置删除元素（从后往前删除）
        for (int i = MpToDelete.size() - 1; i >= 0; --i) 
        {
            if (MpToDelete[i] == 0)continue;
            int index = i;
            Objs.Objects[k].Obj_MPs.erase(Objs.Objects[k].Obj_MPs.begin() + index);
        }
        MpToDelete.clear();




    }

    
}



//剔除外点
void Object::RemoveOutlierD(Object_3D &Obj, float SDnumber)
{
    //1.正态分布剔除2.5个标准差外的点

    // 创建一个用于存储要删除的元素位置的向量
    std::vector<int> MpToDelete;


    // if(Obj.Obj_MPs.size() < 10)continue;//中心极限定理
    CommputeDDistanceSD(Obj);

    Eigen::Vector3f FramePose = Obj.FramePose;
    Eigen::Vector3f Obj_Cen = Obj.Obj_Cen;

    // 遍历向量并记录要删除的元素位置
    for (int i = 0; i < Obj.Obj_MPs.size(); i++)
    {
        // cout<<"compute distance: "<<abs((Obj_Cen - FramePose).norm() - (Obj.Obj_MPs[i]->GetWorldPos() - FramePose).norm())<<endl;
        // cout<<"mean distance: "<<(Obj.mean + 1 * Obj.SD)<<endl;
        if (abs((Obj_Cen - FramePose).norm() - (Obj.Obj_MPs[i]->GetWorldPos() - FramePose).norm()) > (Obj.mean + SDnumber * Obj.SD))
        // if (abs((Obj.Obj_MPs[i]->GetWorldPos() - FramePose).norm()) > (Obj.mean + SDnumber * Obj.SD))
        {
            MpToDelete.push_back(i);
            Obj.Obj_MPs[i]->whetherInObject = false;
        }
    }

    // 根据记录的位置删除元素（从后往前删除）
    for (int i = MpToDelete.size() - 1; i >= 0; i--)
    {   
        if(MpToDelete.empty())continue;
        int index = MpToDelete[i];
        Obj.Obj_MPs.erase(Obj.Obj_MPs.begin() + index);
    }
    MpToDelete.clear();

    

    
}



void Object::IndependentTtest(Object &MPObjs, Object &Objs)
{
    double t = 0;
    for(int i = 0; i < MPObjs.Objects.size(); i++)
    {
        for(int j = 0; j < Objs.Objects.size(); j++)
        {
            cout<<(MPObjs.Objects[i].Obj_Cen - Objs.Objects[j].Obj_Cen).norm()<<endl;
            if((MPObjs.Objects[i].Obj_Cen - Objs.Objects[j].Obj_Cen).norm() >= 0.4)continue;           
            double n1 = MPObjs.Objects[i].Obj_MPs.size();
            if(n1 <= 20)continue; 
            cout<<"n1: "<<MPObjs.Objects[i].Obj_MPs.size()<<endl;
            double n2 = Objs.Objects[j].Obj_MPs.size();
            if(n2 <= 10)continue;           
            cout<<"n2: "<<Objs.Objects[j].Obj_MPs.size()<<endl;
            double X1 = MPObjs.Objects[i].mean;
            cout<<"X1: "<<MPObjs.Objects[i].mean<<endl;
            double X2 = Objs.Objects[j].mean;
            cout<<"X2: "<<Objs.Objects[j].mean<<endl;
            double s1 = MPObjs.Objects[i].SD;
            cout<<"s1: "<<MPObjs.Objects[i].SD<<endl;
            double s2 = Objs.Objects[j].SD;
            cout<<"s1: "<<Objs.Objects[j].SD<<endl;
        
            // t = abs(X1 - X2) / (sqrt(((n1-1)*s1*s1+(n2-1)*s2*s2)/(n1 + n2 -2)) * sqrt(1/n1+1/n2));
            t = (X1 - X2) / (sqrt(((n1 - 1) * s1 * s1 + (n2 - 1) * s2 * s2) / (n1 + n2 - 2)) * sqrt(1 / n1 + 1 / n2));
            cout<<"object Ttestid: "<<MPObjs.Objects[i].mnId<<" "<<Objs.Objects[j].mnId<<endl;
            cout<<"t = "<<t<<endl;
            cout<<"fenzi :"<<abs(X1 - X2)<<endl;
            cout<<"fenmu :"<<(sqrt(((n1 - 1) * s1 * s1 + (n2 - 1) * s2 * s2) / (n1 + n2 - 2)) * sqrt(1 / n1 + 1 / n2))<<endl;
            if(abs(t) <= 1.282)
            {
                AssociateObject(Objs.Objects[j], MPObjs.Objects[i]);
                Objs.Objects.erase(Objs.Objects.begin()+j);
                cout<<"t test association-------------------------------------------------------------------------"<<endl;
            }
        }
    }
    
}


void Object::IndependentDistanceTtest(Object &MPObjs)
{
    double t = 10;
    vector<bool>association(MPObjs.Objects.size(), false);
    for(int i = 0; i < MPObjs.Objects.size(); i++)
    {
        for(int j = 0; j < MPObjs.Objects.size(); j++)
        {
            if(i == j)continue;
            if(association[i] == true)continue;
            cout<<(MPObjs.Objects[i].Obj_Cen - MPObjs.Objects[j].Obj_Cen).norm()<<endl;
            if((MPObjs.Objects[i].Obj_Cen - MPObjs.Objects[j].Obj_Cen).norm() >= 0.4)continue;           
            double n1 = MPObjs.Objects[i].Obj_MPs.size();
            if(n1 <= 20)continue; 
            cout<<"n1: "<<MPObjs.Objects[i].Obj_MPs.size()<<endl;
            double n2 = MPObjs.Objects[j].Obj_MPs.size();
            if(n2 <= 20)continue;           
            cout<<"n2: "<<MPObjs.Objects[j].Obj_MPs.size()<<endl;
            double X1 = MPObjs.Objects[i].mean;
            cout<<"X1: "<<MPObjs.Objects[i].mean<<endl;
            double X2 = MPObjs.Objects[j].mean;
            cout<<"X2: "<<MPObjs.Objects[j].mean<<endl;
            double s1 = MPObjs.Objects[i].SD;
            cout<<"s1: "<<MPObjs.Objects[i].SD<<endl;
            double s2 = MPObjs.Objects[j].SD;
            cout<<"s1: "<<MPObjs.Objects[j].SD<<endl;
        
            // t = abs(X1 - X2) / (sqrt(((n1-1)*s1*s1+(n2-1)*s2*s2)/(n1 + n2 -2)) * sqrt(1/n1+1/n2));
            t = (X1 - X2) / (sqrt(((n1 - 1) * s1 * s1 + (n2 - 1) * s2 * s2) / (n1 + n2 - 2)) * sqrt(1 / n1 + 1 / n2));
            cout<<"object Ttestid: "<<MPObjs.Objects[i].mnId<<" "<<MPObjs.Objects[j].mnId<<endl;
            cout<<"t = "<<t<<endl;
            cout<<"fenzi :"<<abs(X1 - X2)<<endl;
            cout<<"fenmu :"<<(sqrt(((n1 - 1) * s1 * s1 + (n2 - 1) * s2 * s2) / (n1 + n2 - 2)) * sqrt(1 / n1 + 1 / n2))<<endl;
            if(abs(t) <= 1.282)
            {
                AssociateObject(MPObjs.Objects[j], MPObjs.Objects[i]);
                MPObjs.Objects.erase(MPObjs.Objects.begin()+j);
                cout<<"t test association--------------------------------------------------------------------------------------------------------------"<<endl;
                association[i] = true;
            }
            t=10;
        }
    }
    
}


double Object::WhetherInSuperquadric(Object::Object_3D currentObject, Object::Object_3D MapObject)
{   
    int insidenum = 0;
    cv::Mat Twq = MapObject.Twq;
    cv::Mat Twq_inv = Twq.inv();
    float e1 = MapObject.e1;
    float e2 = MapObject.e2;

    for(auto mp : currentObject.Obj_MPs)
    {
        Eigen::Vector3f pos = mp->GetWorldPos(); 
        double euation1 = (Twq_inv.at<float>(0,0)*pos(0) + Twq_inv.at<float>(0,1)*pos(1) + Twq_inv.at<float>(0,2)*pos(2) + Twq_inv.at<float>(0,3)) / MapObject.length;
        double euation2 = (Twq_inv.at<float>(1,0)*pos(0) + Twq_inv.at<float>(1,1)*pos(1) + Twq_inv.at<float>(1,2)*pos(2) + Twq_inv.at<float>(1,3)) / MapObject.width;
        double euation3 = (Twq_inv.at<float>(2,0)*pos(0) + Twq_inv.at<float>(2,1)*pos(1) + Twq_inv.at<float>(2,2)*pos(2) + Twq_inv.at<float>(2,3)) / MapObject.height;
        double F = pow((pow(euation1, 2/e2) + pow(euation2, 2/e2)), e2/e1) + pow(euation3, 2/e1);
        // cout<<"Object ID: "<<MapObject.mnId<<" F: "<<F<<endl;
        if(F<1)
        insidenum++;
    }
    double SUrate = static_cast<double>(insidenum)/currentObject.Obj_MPs.size();
    // cout<<"insidenum : "<<insidenum<<endl;
    // cout<<"rate of SUpoints: "<<SUrate<<endl;

    if(insidenum >= 5) return SUrate;
    else return false;
}



float Object::Calculate3DIoU(Object::Object_3D currentObject, Object::Object_3D MapObject) 
{
    // float intersectionArea = std::max(0.0f, std::min(currentObject.length, MapObject.length))
    //                         * std::max(0.0f, std::min(currentObject.width, MapObject.width))
    //                         * std::max(0.0f, std::min(currentObject.height, MapObject.height));
    // float volumecur = currentObject.length * currentObject.width * currentObject.height;
    // float volumemap = MapObject.length * MapObject.width * MapObject.height;
    // float unionArea = volumecur + volumemap - intersectionArea;
    // float iou = intersectionArea / unionArea;
    // // cout<<"3D IoU: "<<iou<<endl;
    // return iou;


    float xc_max = currentObject.Obj_Cen[0] + currentObject.length / 2;
	float yc_max = currentObject.Obj_Cen[1] + currentObject.width / 2;
	float zc_max = currentObject.Obj_Cen[2] + currentObject.height / 2;
	float xc_min = currentObject.Obj_Cen[0] - currentObject.length / 2;
	float yc_min = currentObject.Obj_Cen[1] - currentObject.width / 2;
	float zc_min = currentObject.Obj_Cen[2] - currentObject.height / 2;

    float xm_max = MapObject.Obj_Cen[0] + MapObject.length / 2;
	float ym_max = MapObject.Obj_Cen[1] + MapObject.width / 2;
	float zm_max = MapObject.Obj_Cen[2] + MapObject.height / 2;
	float xm_min = MapObject.Obj_Cen[0] - MapObject.length / 2;
	float ym_min = MapObject.Obj_Cen[1] - MapObject.width / 2;
	float zm_min = MapObject.Obj_Cen[2] - MapObject.height / 2;

    float inter_x_max = min(xc_max, xm_max);
	float inter_x_min = max(xc_min, xm_min);
	float inter_y_max = min(yc_max, ym_max);
	float inter_y_min = max(yc_min, ym_min);
	float inter_z_max = min(zc_max,zm_max);
	float inter_z_min = max(zc_min,zm_min);

	float inter_w = inter_x_max - inter_x_min;
	float inter_l = inter_y_max - inter_y_min;
	float inter_h = inter_z_max - inter_z_min;

    float interArea = (inter_w < 0 || inter_l < 0 || inter_h < 0) ? 0.0 : inter_w * inter_l * inter_h;
    float area1 = (xc_max - xc_min) * (yc_max - yc_min) * (zc_max - zc_min);
	float area2 = (xm_max - xm_min) * (ym_max - ym_min) * (zm_max - zm_min);
	float unionArea = area1 + area2 - interArea;
	float iou = interArea / unionArea;
	return iou;

}












void Object::KMassociation(Object currentObjects, Object& AllObjects, Object& newobjects)
{


// const int MAXN = 305;
// const int INF = 0x3f3f3f3f;

// int love[currentObjects.Objects.size()][AllObjects.Objects.size()];   // 记录每个妹子和每个男生的好感度
// int ex_Curobj[MAXN];      // 每个妹子的期望值
// int ex_Mapobj[MAXN];       // 每个男生的期望值
// bool vis_Curobj[MAXN];    // 记录每一轮匹配匹配过的女生
// bool vis_Mapobj[MAXN];     // 记录每一轮匹配匹配过的男生
// int match[MAXN];        // 记录每个男生匹配到的妹子 如果没有则为-1
// int slack[MAXN];        // 记录每个汉子如果能被妹子倾心最少还需要多少期望值

// int N;

KMalgorithm KM(currentObjects.Objects.size(), AllObjects.Objects.size());

currentObjects.ComputeObjectCenter();//算一下新建的物体质心

//1.计算权重
    //1.1计算地图点权重
    // 定义地图点比较函数
    struct MapPointComparator 
    {
        bool operator()(const MapPoint* mp1, const MapPoint* mp2) const 
        {
            return mp1->mnId < mp2->mnId;
        }
    };

    for(int i = 0; i < AllObjects.Objects.size(); i++)//所有地图中的物体
    {       
        for(int j = 0; j < currentObjects.Objects.size(); j++)//所有当前帧的物体
        {
            if(currentObjects.Objects[j].Obj_MPs.size() < 5)continue;
            int samepoints = 0;
            set<MapPoint*, MapPointComparator> AllMappoints(AllObjects.Objects[i].Obj_MPs.begin(), AllObjects.Objects[i].Obj_MPs.end());
            set<MapPoint*, MapPointComparator> CurrentMappoints(currentObjects.Objects[j].Obj_MPs.begin(), currentObjects.Objects[j].Obj_MPs.end());

            for (const auto& mp : CurrentMappoints) 
            {
                if (AllMappoints.count(mp) > 0) 
                {
                    cout << "Found MapPoint with mnId " << mp->mnId << endl;
                    samepoints++;
                }
            }

            double MPrate = static_cast<double>(samepoints)/currentObjects.Objects[j].Obj_MPs.size();
            cout<<"rate of mappoint :"<<MPrate<<endl;

            AllMappoints.clear();
            CurrentMappoints.clear();


            //1.2计算类别权重
            // float CenterDistance = (currentObjects.Objects[j].Obj_Cen - AllObjects.Objects[i].Obj_Cen).norm();
            double CLrate = 0;
            if(currentObjects.Objects[j].cid == AllObjects.Objects[i].cid)
            CLrate += currentObjects.Objects[j].Score;
            // CLrate += 0.5*currentObjects.Objects[j].Score;
            // if(CenterDistance < 0.5)
            // CLrate += 0.5*(1-(2*CenterDistance));
            cout<<"rate of class :"<<CLrate<<endl;



            // //1.2计算类别权重
            // float CenterDistance = (currentObjects.Objects[j].Obj_Cen - AllObjects.Objects[i].Obj_Cen).norm();
            // double CLrate = 0;
            // if(currentObjects.Objects[j].cid == AllObjects.Objects[i].cid)
            // CLrate += currentObjects.Objects[j].Score;
            // // if(CenterDistance < 0.5)
            // // CLrate += 0.5*(1-(2*CenterDistance));
            // // if(samepoints>=0)
            // // CLrate += 0.1*static_cast<double>(samepoints)/currentObjects.Objects[j].Obj_MPs.size();
            // cout<<"rate of class :"<<CLrate<<endl;


            //1.3计算IoU权重
            cv::Rect rectI = AllObjects.Objects[i].Obj_box & currentObjects.Objects[j].Obj_box;
            cv::Rect rectU = AllObjects.Objects[i].Obj_box | currentObjects.Objects[j].Obj_box;
            // double rectU = AllObjects.Objects[i].Obj_box.area() + currentObjects.Objects[j].Obj_box.area() - rectI.area();
            double IoUrate2D = rectI.area() *1.0/ rectU.area();
            // double IoUrate2D = rectI.area() *1.0/ (rectU);
            double IoUrate3D = Calculate3DIoU(currentObjects.Objects[j], AllObjects.Objects[i]);
            double IoUrate = 0.5*IoUrate2D + 0.5*IoUrate3D;
            // double IoUrate = IoUrate2D;
            // cout<<"Mapobject mnid:"<<AllObjects.Objects[i].mnId<<" box:"<<AllObjects.Objects[i].Obj_box<<endl;
            // cout<<"Newobject mnid:"<<currentObjects.Objects[j].mnId<<" box:"<<currentObjects.Objects[j].Obj_box<<endl;
            cout<<"rete of IoU :"<<IoUrate<<" 2D:"<<IoUrate2D<<" 3D:"<<IoUrate3D<<endl;

            //1.4计算地图点是否在物体内的权重
            double SUrate = 0;
            // SUrate = 0.8*WhetherInSuperquadric(currentObjects.Objects[j], AllObjects.Objects[i]) + 0.2*Calculate3DIoU(currentObjects.Objects[j], AllObjects.Objects[i]);
            SUrate = WhetherInSuperquadric(currentObjects.Objects[j], AllObjects.Objects[i]);
            cout<<"rate of SUpoints: "<<SUrate<<endl;

            //1.5计算总权重，百分制
            KM.love[j][i] = static_cast<int>(100*(0.3*MPrate + 0.2*CLrate + 0.2*IoUrate + 0.3*SUrate));
            // KM.love[j][i] = static_cast<int>(100*(0*MPrate + 0*CLrate + 0*IoUrate + 1*SUrate));
            // cout<<"Total rate : "<<j<<" "<<i<<" "<<KM.love[j][i];
            // if(KM.love[j][i] <= 10)KM.love[j][i] = 0;
        }
    }

 

//2.KM算法计算最优匹配
    cout<<"KM algorithm: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    KMcalculate(KM);    

//3.匹配完成 关联或新建物体
    //关联物体
    vector<bool> associated(currentObjects.Objects.size(), false);
    for(int i = 0; i < KM.match.size(); i++)
    {
        if(KM.match[i] != -1)
        {
            associated[KM.match[i]] = true;
            AssociateObject(currentObjects.Objects[KM.match[i]], AllObjects.Objects[i]);
            AllObjects.Objects[i].WhetherInFrame = true;

            //剔除外点
            Object::RemoveOutlierD(AllObjects.Objects[i], 2.5);

        }
    }
    for(int j = 0; j < associated.size(); j++)
    {
        cout<<"association :"<<j<<" "<<associated[j]<<endl;
    }

    //添加新物体
    for(int i = 0; i < currentObjects.Objects.size(); i++)
    {
        cout<<i<<" associated is "<<associated[i]<<endl;
        if(associated[i] == true)continue;
        if(currentObjects.Objects[i].Obj_MPs.size() <= 5)continue;//地图点少于5的物体不要
        newobjects.Objects.push_back(currentObjects.Objects[i]);
        cout<<"newobjects in object"<<newobjects.Objects.size()<<endl;
    }


}


bool Object::dfs(int Curobj, KMalgorithm &KM, int count)
{   
    if(count >=20)return true;
    KM.vis_Curobj[Curobj] = true;

    for (int Mapobj = 0; Mapobj < KM.love[Curobj].size(); Mapobj++) 
    {

        if (KM.vis_Mapobj[Mapobj]) continue; // 每一轮匹配 每个男生只尝试一次

        if(KM.love[Curobj][Mapobj] <= 20)continue;
        int gap = KM.ex_Curobj[Curobj] + KM.ex_Mapobj[Mapobj] - KM.love[Curobj][Mapobj];
        // cout<<"KM.love[Curobj].size() :"<<KM.love[Curobj].size()<<endl;
        // cout<<"KM.ex_Curobj[Curobj] :"<<KM.ex_Curobj[Curobj]<<endl;
        // cout<<"KM.ex_Mapobj[Mapobj] :"<<KM.ex_Mapobj[Mapobj]<<endl;
        // cout<<"KM.love[Curobj][Mapobj] :"<<KM.love[Curobj][Mapobj]<<endl;
        // cout<<"gap :"<<gap<<endl;

        if (gap == 0) {  // 如果符合要求
            KM.vis_Mapobj[Mapobj] = true;
            if (KM.match[Mapobj] == -1 || dfs(KM.match[Mapobj], KM, count)) 
            {    // 找到一个没有匹配的男生 或者该男生的妹子可以找到其他人
                KM.match[Mapobj] = Curobj;
                return true;
            }
        } else {
            KM.slack[Mapobj] = min(KM.slack[Mapobj], gap);  // slack 可以理解为该男生要得到女生的倾心 还需多少期望值 取最小值 备胎的样子【捂脸
        }
    }

    return false;
}

void Object::KMcalculate(KMalgorithm &KM)
{
    fill(KM.match.begin(), KM.match.end(), -1);// 初始每个男生都没有匹配的女生
    // memset(KM.match, -1, sizeof match);  
    fill(KM.ex_Mapobj.begin(), KM.ex_Mapobj.end(), 0);// 初始每个男生的期望值为0  
    // memset(ex_Mapobj, 0, sizeof ex_Mapobj);   
    // cout<<1<<endl;
    // 每个女生的初始期望值是与她相连的男生最大的好感度
    for (int i = 0; i < KM.love.size(); ++i) 
    {
        KM.ex_Curobj[i] = KM.love[i][0];
        for (int j = 1; j < KM.love[i].size(); ++j) 
        {
            KM.ex_Curobj[i] = max(KM.ex_Curobj[i], KM.love[i][j]);
        }
    }
    // cout<<2<<endl;


    // 尝试为每一个女生解决归宿问题
    for (int i = 0; i < KM.love.size(); ++i) {

        fill(KM.slack.begin(), KM.slack.end(), 100);    // 因为要取最小值 初始化为无穷大

        int count = 0;
        while (1) 
        {
            // 为每个女生解决归宿问题的方法是 ：如果找不到就降低期望值，直到找到为止

            // 记录每轮匹配中男生女生是否被尝试匹配过
            fill(KM.vis_Curobj.begin(), KM.vis_Curobj.begin(), false);
            fill(KM.vis_Mapobj.begin(), KM.vis_Mapobj.begin(), false);
            // memset(vis_Curobj, false, sizeof vis_Curobj);
            // memset(vis_Mapobj, false, sizeof vis_Mapobj);

            if (dfs(i, KM, count)) break;  // 找到归宿 退出
            count++;
            // cout<<3<<" "<<count<<endl;

            // 如果不能找到 就降低期望值
            // 最小可降低的期望值
            int d = 1;
            for (int j = 0; j < KM.love.size(); ++j)
                if (!KM.vis_Mapobj[j]) d = min(d, KM.slack[j]);

            for (int j = 0; j < KM.love.size(); ++j) 
            {
                // 所有访问过的女生降低期望值
                if (KM.vis_Curobj[j]) 
                {                
                    KM.ex_Curobj[j] -= d;
                    cout<<"Curobj expect :"<<KM.ex_Curobj[j]<<endl;
                    if(KM.ex_Curobj[j] < 0)break;
                }

                // 所有访问过的男生增加期望值
                if (KM.vis_Mapobj[j]) 
                {
                    KM.ex_Mapobj[j] += d;
                    cout<<"Mapobj expect :"<<KM.ex_Mapobj[j]<<endl;
                }
                // 没有访问过的Mapobj 因为Curobj们的期望值降低，距离得到女生倾心又进了一步！
                else 
                {
                    KM.slack[j] -= d;
                    cout<<"Mapobj slack :"<<KM.slack[j]<<endl;
                }
            }

        }
    }
    // cout<<4<<endl;

    // for (int i = 0; i < KM.match.size(); ++i)
    //     cout<<"match : "<<i<<"-"<<KM.match[i]<<endl;

}

void Object::ComputeRotation(Object::Object_3D& Object, Frame currentFrame, vector<cv::Vec4f> lines)
{


    //计算物体坐标轴   
    //定义坐标系的原点和坐标轴向量
    //重力方向
    double angleG = Object.rotationX * M_PI / 180.0f;
    float angleR = Object.rotationY * M_PI / 180.0f;
    Eigen::Matrix3f RG_Eigen;
    RG_Eigen <<  1, 0, 0,
                0, cos(angleG), -sin(angleG),
                0, sin(angleG), cos(angleG);

    Eigen::Matrix3f RR_Eigen;
    RR_Eigen << cos(angleR), 0, sin(angleR),
                0, 1, 0,
                -sin(angleR), 0, cos(angleR);





    
    // 定义坐标系的原点和坐标轴向量
    Eigen::Vector3f Obj3Dcenter = Object.Obj_Cen; 



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

    // for(auto line :lines)
    // cout<<"line data: "<<line<<endl;

    vector<cv::Vec4f> lines_inmask;

    //判断直线是否在mask里，opencv自带函数
    for(int i = 0; i < lines.size(); i++)
    {
        cv::Point point1(lines[i][0], lines[i][1]);  
        cv::Point point2(lines[i][2], lines[i][3]);  
        if((cv::pointPolygonTest(Object.mask, point1, false) >= 0) && (cv::pointPolygonTest(Object.mask, point2, false) >= 0))
        {
            lines_inmask.push_back(lines[i]);
        }
    }

    // //判断直线是否平行
    // //定义一个阈值，用于判断斜率是否相近，根据需要进行调整
    // const double slopeThreshold = 0.1;
    // std::vector<cv::Vec4f> parallelLines;
    // if(lines_inmask.size() > 0)
    // {
    //     for (size_t i = 0; i < lines_inmask.size(); ++i) 
    //     {
    //         cv::Vec4f line1 = lines_inmask[i];
    //         double slope1 = calculateSlope(line1);

    //         // 查找与当前线段平行的其他线段
    //         for (size_t j = i + 1; j < lines_inmask.size(); ++j) 
    //         {
    //             cv::Vec4f line2 = lines_inmask[j];
    //             double slope2 = calculateSlope(line2);

    //             // 判断斜率是否相近
    //             if (std::abs(slope1 - slope2) < slopeThreshold) 
    //             {
    //                 parallelLines.push_back(line1);
    //                 // parallelLines.push_back(line2);
    //             }
    //         }
    //     }
    // }

    double minabgle = 200;
    double minabgleX = 200;
    double minabgleZ = 200;
    //计算夹角
    for(int i = 0; i < lines_inmask.size(); i++)
    {
        cv::Point2f point1(lines_inmask[i][0], lines_inmask[i][1]);  
        cv::Point2f point2(lines_inmask[i][2], lines_inmask[i][3]); 
        double angleX = CalculateAngle(Obj2Dcenter, imgXAxis, point1, point2);
        double angleZ = CalculateAngle(Obj2Dcenter, imgZAxis, point1, point2);
        double angleY = CalculateAngle(Obj2Dcenter, imgYAxis, point1, point2);
        cout<<"lines XZY :"<<Object.mnId<<" "<<angleX<<" "<<angleZ<<" "<<angleY<<endl;
        if(abs(abs(angleY) - 90) < 30)
        {
            if(angleX <= 30 && angleX > 0)minabgleX = angleX;
            if(angleX > 150 && angleX <= 180)minabgleX = angleX - 180;
            if(angleX <= 0 && angleX > -30)minabgleX = angleX;
            if(angleX <= -150 && angleX > -180)minabgleX = 180 + angleX;
        }
        if(abs(abs(angleZ) - 90) < 30)
        {
            if(angleZ <= 30 && angleZ > 0)minabgleZ = angleZ;
            if(angleZ > 150 && angleZ <= 180)minabgleZ = angleZ - 180;
            if(angleZ <= 0 && angleZ > -30)minabgleZ = angleZ;
            if(angleZ <= -150 && angleZ > -180)minabgleZ = 180 + angleZ;
        }

        if(abs(minabgleX) < abs(minabgle))minabgle = minabgleX;
        if(abs(minabgleZ) < abs(minabgle))minabgle = minabgleZ;
        if(minabgle != 200 )Object.rotationY = minabgle;

    }

    // double minabgle = 200;
    // //计算夹角
    // for(int i = 0; i < lines_inmask.size(); i++)
    // {
    //     cv::Point2f point1(lines_inmask[i][0], lines_inmask[i][1]);  
    //     cv::Point2f point2(lines_inmask[i][2], lines_inmask[i][3]); 
    //     double angleX = CalculateAngle(Obj2Dcenter, imgXAxis, point1, point2);
    //     double angleZ = CalculateAngle(Obj2Dcenter, imgZAxis, point1, point2);
    //     cout<<"lines XZ :"<<Object.mnId<<" "<<angleX<<" "<<angleZ<<endl;
    //     if(abs(angleX) < abs(minabgle))minabgle = angleX;
    //     if(abs(angleZ) < abs(minabgle))minabgle = angleZ;
    //     if(minabgle != 200)Object.rotationY = minabgle;
    //     cout<<"object angle in compute: "<<Object.rotationY<<endl;

    // }


    

}

double CalculateAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& q1, const cv::Point& q2)
{
    cv::Point vec1 = p2 - p1;  // p1指向p2
    cv::Point vec2 = q2 - q1;

    double length1 = cv::norm(vec1);
    double length2 = cv::norm(vec2);

    double dotProduct = vec1.dot(vec2);
    double crossProduct = vec1.x * vec2.y - vec1.y * vec2.x;  // 叉积计算

    double angleRad = std::acos(dotProduct / (length1 * length2));

    // 使用叉积判断夹角的方向
    if (crossProduct < 0)
        angleRad = -angleRad;

    double angleDeg = angleRad * (180.0 / CV_PI);

    return angleDeg;
}


// 判断两个线段是否平行
bool areLinesParallel(const cv::Vec4f& line1, const cv::Vec4f& line2) {
    // 线段1的斜率
    double slope1 = (line1[3] - line1[1]) / (line1[2] - line1[0]);

    // 线段2的斜率
    double slope2 = (line2[3] - line2[1]) / (line2[2] - line2[0]);

    // 定义一个阈值来容忍浮点数误差
    double epsilon = 0.01;

    // 判断两个斜率是否接近（即线段是否平行）
    if (std::abs(slope1 - slope2) < epsilon) {
        return true;
    }

    return false;
}

// 计算直线的斜率
double calculateSlope(const cv::Vec4f& line) 
{
    // 计算斜率 (dy / dx)
    double dx = line[2] - line[0];
    double dy = line[3] - line[1];
    if (std::abs(dx) < 1e-6) {
        // 处理垂直线的情况，斜率设为无穷大
        return std::numeric_limits<double>::infinity();
    }
    return dy / dx;
}








    

}
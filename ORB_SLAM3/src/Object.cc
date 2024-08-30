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


//Initial object each keyframe
Object Object::InitialObject(Frame currentFrame, YOLO currentYolo)
{
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> mvbMap, mvbVO;

    vector<int>cidVec;
    vector<vector<cv::KeyPoint>>Obj_KPsVec(currentYolo.yoloLines.size());
    vector<vector<MapPoint*>>Obj_MPsVec(currentYolo.yoloLines.size());

    vCurrentKeys = currentFrame.mvKeysUn;
    
    int N = vCurrentKeys.size();
    cout<<"Keypoint size:"<< N <<endl;

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    //Retrieve information about mappoints in the current keyframe.
    for(int i = 0; i < N; i++)
    {
        MapPoint* pMP = currentFrame.mvpMapPoints[i];
        if(pMP)
        {
            if(!currentFrame.mvbOutlier[i])
            {
                if(pMP->Observations()>0)
                    mvbMap[i]=true;
                else
                    mvbVO[i]=true;
            }
        }
    }

    //Determine if the feature point is on the object.
    for(int i = 0; i < N; i++)
    {
        if(mvbVO[i] || mvbMap[i])
        {
            for(int j = 0; j < currentYolo.yoloLines.size(); j++)//Process YOLO information.
            {   
                if(currentYolo.yoloLines[j].mask.size() <= 4)continue;
                if(stoi(currentYolo.yoloLines[j].cla) == 60 ||
                   stoi(currentYolo.yoloLines[j].cla) == 72 ||
                   stoi(currentYolo.yoloLines[j].cla) == 32 ||
                   stoi(currentYolo.yoloLines[j].cla) == 0 || 
                   stoi(currentYolo.yoloLines[j].cla) == 15 ||
                   stoi(currentYolo.yoloLines[j].cla) == 63 ||
                   stoi(currentYolo.yoloLines[j].cla) == 68 ||
                   stoi(currentYolo.yoloLines[j].cla) == 58)continue;//Remove tables, plants, refrigerators, sports balls, people, cats, laptops, and microwaves.
                if(currentYolo.yoloLines[j].cT < 0.7)continue;//Remove objects with a YOLO confidence score less than 0.5.

                //Determine if a point is inside the bounding box.
                // double box_leftx = currentYolo.yoloLines[j].box.x - currentYolo.yoloLines[j].box.width/2;
                // double box_rightx = currentYolo.yoloLines[j].box.x + currentYolo.yoloLines[j].box.width/2;
                // double box_lefty = currentYolo.yoloLines[j].box.y - currentYolo.yoloLines[j].box.height/2;
                // double box_righty = currentYolo.yoloLines[j].box.y + currentYolo.yoloLines[j].box.height/2;

                // double box_leftx = currentYolo.yoloLines[j].box.x;
                // double box_rightx = currentYolo.yoloLines[j].box.x + currentYolo.yoloLines[j].box.width;
                // double box_lefty = currentYolo.yoloLines[j].box.y;
                // double box_righty = currentYolo.yoloLines[j].box.y + currentYolo.yoloLines[j].box.height;

                // if(box_leftx < 5 || box_lefty < 5 || box_rightx > 675 || box_righty > 475)continue;

                // if(vCurrentKeys[i].pt.x >= box_leftx && vCurrentKeys[i].pt.x <= box_rightx)
                // {
                //     if(vCurrentKeys[i].pt.y >= box_lefty && vCurrentKeys[i].pt.y <= box_righty)
                //     {
                //         //Determine if a point is inside a mask.
                //         cv::Point point(vCurrentKeys[i].pt.x, vCurrentKeys[i].pt.y);                   
                //         if(cv::pointPolygonTest(currentYolo.yoloLines[j].mask, point, false) >= 0)
                //         {   
                //             //Add feature points/mappoint information for this object.
                //             Obj_KPsVec[j].push_back(vCurrentKeys[i]);
                //             currentFrame.mvpMapPoints[i]->whetherInObject = true;
                //             Obj_MPsVec[j].push_back(currentFrame.mvpMapPoints[i]);
                //         }
                       
                //     }
                // }


                // 获取边界框的左上角和右下角坐标
                double box_leftx = currentYolo.yoloLines[j].box.x;
                double box_rightx = currentYolo.yoloLines[j].box.x + currentYolo.yoloLines[j].box.width;
                double box_lefty = currentYolo.yoloLines[j].box.y;
                double box_righty = currentYolo.yoloLines[j].box.y + currentYolo.yoloLines[j].box.height;

                // 跳过超出图像边界的边界框
                if (box_leftx < 5 || box_lefty < 5 || box_rightx > 675 || box_righty > 475) continue;

                // 调整掩码点，使其相对于边界框
                std::vector<cv::Point> adjustedMask;
                for (const auto& point : currentYolo.yoloLines[j].mask) {
                    // 计算掩码点相对于 bbox 的坐标
                    adjustedMask.push_back(cv::Point(point.x + box_leftx, point.y + box_lefty));
                }

                // 更新检测特征点是否在边界框和掩码内的逻辑
                if (vCurrentKeys[i].pt.x >= box_leftx && vCurrentKeys[i].pt.x <= box_rightx) {
                    if (vCurrentKeys[i].pt.y >= box_lefty && vCurrentKeys[i].pt.y <= box_righty) {
                        // 创建特征点对象
                        cv::Point point(vCurrentKeys[i].pt.x, vCurrentKeys[i].pt.y);                   

                        // 判断特征点是否在调整后的掩码内
                        if (cv::pointPolygonTest(adjustedMask, point, false) >= 0) {   
                            // 将特征点和地图点信息添加到该对象
                            Obj_KPsVec[j].push_back(vCurrentKeys[i]);
                            currentFrame.mvpMapPoints[i]->whetherInObject = true;
                            Obj_MPsVec[j].push_back(currentFrame.mvpMapPoints[i]);
                        }
                    }
                }

                            
            }
        }
    }
    
    //Add confidence score information.
    for(int j = 0; j < currentYolo.yoloLines.size(); j++)
    {
        cidVec.push_back(stoi(currentYolo.yoloLines[j].cla));
        cout<<"Obj_KPsVec size:"<<Obj_KPsVec[j].size()<<endl;

    }


    //Create an object class.
    Object curfObject(cidVec, Obj_KPsVec, Obj_MPsVec);

    for(int k = 0; k < curfObject.Objects.size(); k++)
    {
        //initial object information
        curfObject.Objects[k].Obj_Fms.push_back(currentFrame.mnId);
        curfObject.Objects[k].FrameId = currentFrame.mnId;
        curfObject.Objects[k].FramePose = currentFrame.GetCameraCenter();
        curfObject.Objects[k].Obj_box = cv::Rect(currentYolo.yoloLines[k].box.x - currentYolo.yoloLines[k].box.width/2, currentYolo.yoloLines[k].box.y - currentYolo.yoloLines[k].box.height/2, 
                                                currentYolo.yoloLines[k].box.width, currentYolo.yoloLines[k].box.height);

        
        curfObject.Objects[k].mask = currentYolo.yoloLines[k].mask;
        curfObject.Objects[k].WhetherInFrame = true;
        curfObject.Objects[k].Score = currentYolo.yoloLines[k].cT;

        //Determine object shape parameters.
        if(curfObject.Objects[k].cid == 13 || curfObject.Objects[k].cid == 26 || curfObject.Objects[k].cid == 28 || curfObject.Objects[k].cid == 30 
        || curfObject.Objects[k].cid == 31 || curfObject.Objects[k].cid == 33 || curfObject.Objects[k].cid == 36 || curfObject.Objects[k].cid == 37 
        || curfObject.Objects[k].cid == 56 || curfObject.Objects[k].cid == 57 || curfObject.Objects[k].cid == 59 || curfObject.Objects[k].cid == 60 
        || curfObject.Objects[k].cid == 62 || curfObject.Objects[k].cid == 63 || curfObject.Objects[k].cid == 64 || curfObject.Objects[k].cid == 65
        || curfObject.Objects[k].cid == 66 || curfObject.Objects[k].cid == 67 || curfObject.Objects[k].cid == 68 || curfObject.Objects[k].cid == 69
        || curfObject.Objects[k].cid == 70 || curfObject.Objects[k].cid == 71 || curfObject.Objects[k].cid == 72 || curfObject.Objects[k].cid == 73)
        {
            curfObject.Objects[k].e1 = 0.1;  
            curfObject.Objects[k].e2 = 0.1;  
        }
        else if(curfObject.Objects[k].cid == 10 || curfObject.Objects[k].cid == 39|| curfObject.Objects[k].cid == 40 || curfObject.Objects[k].cid == 41
        || curfObject.Objects[k].cid == 45 || curfObject.Objects[k].cid == 54 || curfObject.Objects[k].cid == 55 || curfObject.Objects[k].cid == 75)
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


    //delete bad objects
    std::vector<int> indicesToDelete;
    for (int k = 0; k < curfObject.Objects.size(); k++)
    {   
        curfObject.Objects[k].WhetherInFrame = false;

        int MPsize = curfObject.Objects[k].Obj_MPs.size();

        if (MPsize <= 10) //mappoints is less than 10
        {
            indicesToDelete.push_back(k);
            for(int l = 0; l < MPsize; l++)
            {
                curfObject.Objects[k].Obj_MPs[l]->whetherInObject = false;
            }
        }
    }
    for (int i = indicesToDelete.size() - 1; i >= 0; i--)
    {   
        if(indicesToDelete.empty())continue;
        int index = indicesToDelete[i];
        curfObject.Objects.erase(curfObject.Objects.begin() + index);
    }

    //delete objects which IoU score is too big
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

                //IoU
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



    //initial length width height
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

            //calculate
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



            //Calculate the rotation matrix, manually set the direction of gravity.
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

    return curfObject;

}



//did not use
bool Object::InPolygon (cv::Point p, vector<cv::Point>& Polygon)  
{  


	int nCross = 0;  
	for (int i = 0; i < Polygon.size(); i++)   
	{  
		cv::Point p1 = Polygon[i];  
		cv::Point p2 = Polygon[(i + 1) % Polygon.size()];
 
		if ( p1.y == p2.y )  
			continue;  
		if ( p.y < min(p1.y, p2.y) )  
			continue;  
		if ( p.y >= max(p1.y, p2.y) )  
			continue;  
 
		double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;  
 
		if ( x > p.x )  
		{  
			nCross++;  
		}  
 
	}  
 

	if ((nCross % 2) == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
} 

//Compute object center
void Object::ComputeObjectCenter()
{

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






//did not use
void Object::Association(Object currentObjects, Object& AllObjects, Object& newobjects)
{
    currentObjects.ComputeObjectCenter();

    struct MapPointComparator 
    {
        bool operator()(const MapPoint* mp1, const MapPoint* mp2) const 
        {
            return mp1->mnId < mp2->mnId;
        }
    };

    vector<bool> associated(currentObjects.Objects.size(), false);

    for(int i = 0; i < AllObjects.Objects.size(); i++)
    {       
        // cout<<"Objects_map size:"<<AllObjects.Objects.size()<<endl;
        for(int j = 0; j < currentObjects.Objects.size(); j++)
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

            if(samepoints >= 10 && associated[j] == false)
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



//Associate object
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

    //add different mappoints
    for(int m = 0; m < difMappoints.size(); m++)
    {
        //change mappoint id
        difMappoints[m]->ObjectId = MapObject.mnId;
        MapObject.Obj_MPs.push_back(difMappoints[m]);
    }

    //add frame
    MapObject.Obj_Fms.push_back(currentObject.Obj_Fms.back());
    MapObject.FramePose = currentObject.FramePose;

    //add class
    MapObject.cid = currentObject.cid;


    //add mask and bbox
    MapObject.Obj_box = currentObject.Obj_box;
    MapObject.mask = currentObject.mask;

    difMappoints.clear();
    AllMappoints.clear();
    CurrentMappoints.clear();

}


//did not use, for test
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

//did not use, for test
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

//Calculate the Euclidean distance used for outlier rejection.
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

//did not use, for test
void Object::RemoveOutlier(Object &Objs)
{
    std::vector<int> MpToDelete;

    for(int k = 0; k < Objs.Objects.size(); k++)
    {
        if(Objs.Objects[k].Obj_MPs.size() < 30)continue;
        CommputeDistanceSD(Objs.Objects[k]);
        for (int i = 0; i < Objs.Objects[k].Obj_MPs.size(); i++)
        {

            if ((Objs.Objects[k].Obj_Cen - Objs.Objects[k].Obj_MPs[i]->GetWorldPos()).norm() > (Objs.Objects[k].mean + 3 * Objs.Objects[k].SD))
            {
                MpToDelete.push_back(i);
                Objs.Objects[k].Obj_MPs[i]->whetherInObject = false;
            }
        }

        for (int i = MpToDelete.size() - 1; i >= 0; i--)
        {   
            if(MpToDelete.empty())continue;
            int index = MpToDelete[i];
            Objs.Objects[k].Obj_MPs.erase(Objs.Objects[k].Obj_MPs.begin() + index);
        }
        MpToDelete.clear();

    }

    
}


//did not use, for test
void Object::RemoveOutlierXYZ(Object &Objs)
{

    for(int k = 0; k < Objs.Objects.size(); k++)
    {

        if(Objs.Objects[k].Obj_MPs.size() < 30)continue;

        std::vector<int> MpToDelete(Objs.Objects[k].Obj_MPs.size(), 0);

        CommputeDistanceSDXYZ(Objs.Objects[k]);
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

        for (int i = MpToDelete.size() - 1; i >= 0; --i) 
        {
            if (MpToDelete[i] == 0)continue;
            int index = i;
            Objs.Objects[k].Obj_MPs.erase(Objs.Objects[k].Obj_MPs.begin() + index);
        }
        MpToDelete.clear();

    }
   
}



//outlier rejection, Eq.2-3 in paper.
void Object::RemoveOutlierD(Object_3D &Obj, float SDnumber)
{
    //Reject points beyond 2.5 standard deviations.
    std::vector<int> MpToDelete;
    CommputeDDistanceSD(Obj);

    Eigen::Vector3f FramePose = Obj.FramePose;
    Eigen::Vector3f Obj_Cen = Obj.Obj_Cen;

    for (int i = 0; i < Obj.Obj_MPs.size(); i++)
    {

        if (abs((Obj_Cen - FramePose).norm() - (Obj.Obj_MPs[i]->GetWorldPos() - FramePose).norm()) > (Obj.mean + SDnumber * Obj.SD))
        {
            MpToDelete.push_back(i);
            Obj.Obj_MPs[i]->whetherInObject = false;
        }
    }

    for (int i = MpToDelete.size() - 1; i >= 0; i--)
    {   
        if(MpToDelete.empty())continue;
        int index = MpToDelete[i];
        Obj.Obj_MPs.erase(Obj.Obj_MPs.begin() + index);
    }
    MpToDelete.clear();
    
}


//did not use, for test
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

//did not use, for test
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

//Whether a mappoint in superquadric.
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
        double F = pow((pow(euation1, 2/e2) + pow(euation2, 2/e2)), e2/e1) + pow(euation3, 2/e1); //Eq.1 in paper. More details can be found in [17].
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


//3D IoU score
float Object::Calculate3DIoU(Object::Object_3D currentObject, Object::Object_3D MapObject) 
{

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



//KM assocaition algorithm
void Object::KMassociation(Object currentObjects, Object& AllObjects, Object& newobjects)
{



KMalgorithm KM(currentObjects.Objects.size(), AllObjects.Objects.size());

currentObjects.ComputeObjectCenter();

//1.calculate score
    //1.1calculate mappoint score
    struct MapPointComparator 
    {
        bool operator()(const MapPoint* mp1, const MapPoint* mp2) const 
        {
            return mp1->mnId < mp2->mnId;
        }
    };

    for(int i = 0; i < AllObjects.Objects.size(); i++)//all objects in map
    {       
        for(int j = 0; j < currentObjects.Objects.size(); j++)//all objects in current frame
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

            double MPrate = static_cast<double>(samepoints)/currentObjects.Objects[j].Obj_MPs.size();//Eq.5 in paper.
            cout<<"rate of mappoint :"<<MPrate<<endl;

            AllMappoints.clear();
            CurrentMappoints.clear();


            //1.2calculate class score
            double CLrate = 0;
            if(currentObjects.Objects[j].cid == AllObjects.Objects[i].cid)
            CLrate += currentObjects.Objects[j].Score;//Eq.6 in paper.
            cout<<"rate of class :"<<CLrate<<endl;

            //1.3calculate IoU score
            cv::Rect rectI = AllObjects.Objects[i].Obj_box & currentObjects.Objects[j].Obj_box;
            cv::Rect rectU = AllObjects.Objects[i].Obj_box | currentObjects.Objects[j].Obj_box;
            double IoUrate2D = rectI.area() *1.0/ rectU.area();
            double IoUrate3D = Calculate3DIoU(currentObjects.Objects[j], AllObjects.Objects[i]);
            double IoUrate = 0.5*IoUrate2D + 0.5*IoUrate3D;//Eq.7 in paper.
            cout<<"rete of IoU :"<<IoUrate<<" 2D:"<<IoUrate2D<<" 3D:"<<IoUrate3D<<endl;

            //1.4calculate superquadric score
            double SUrate = 0;
            SUrate = WhetherInSuperquadric(currentObjects.Objects[j], AllObjects.Objects[i]);//Eq.8 in paper.
            cout<<"rate of SUpoints: "<<SUrate<<endl;

            //1.5Calculate total score, in percentage.
            KM.love[j][i] = static_cast<int>(100*(0.3*MPrate + 0.2*CLrate + 0.2*IoUrate + 0.3*SUrate));//Eq.4 in paper.
        }
    }

 

//2.Use the KM algorithm to compute the optimal match.
    cout<<"KM algorithm: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    KMcalculate(KM);    

//3.Matching completed. Associate or create new objects.
    //Associate
    vector<bool> associated(currentObjects.Objects.size(), false);
    for(int i = 0; i < KM.match.size(); i++)
    {
        if(KM.match[i] != -1)
        {
            associated[KM.match[i]] = true;
            AssociateObject(currentObjects.Objects[KM.match[i]], AllObjects.Objects[i]);
            AllObjects.Objects[i].WhetherInFrame = true;

            //outlier rejection
            Object::RemoveOutlierD(AllObjects.Objects[i], 2.5);

        }
    }
    for(int j = 0; j < associated.size(); j++)
    {
        cout<<"association :"<<j<<" "<<associated[j]<<endl;
    }

    //create new objects
    for(int i = 0; i < currentObjects.Objects.size(); i++)
    {
        cout<<i<<" associated is "<<associated[i]<<endl;
        if(associated[i] == true)continue;
        if(currentObjects.Objects[i].Obj_MPs.size() <= 5)continue;//do not create object which mappoints less than 5
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

        if (KM.vis_Mapobj[Mapobj]) continue; 

        if(KM.love[Curobj][Mapobj] <= 20)continue;
        int gap = KM.ex_Curobj[Curobj] + KM.ex_Mapobj[Mapobj] - KM.love[Curobj][Mapobj];

        if (gap == 0) {  
            KM.vis_Mapobj[Mapobj] = true;
            if (KM.match[Mapobj] == -1 || dfs(KM.match[Mapobj], KM, count)) 
            {    
                KM.match[Mapobj] = Curobj;
                return true;
            }
        } else {
            KM.slack[Mapobj] = min(KM.slack[Mapobj], gap);  
        }
    }

    return false;
}

//KM algorithm
//Thanks for the great work: http://www.cnblogs.com/wenruo/p/5264235.html
void Object::KMcalculate(KMalgorithm &KM)
{
    fill(KM.match.begin(), KM.match.end(), -1); 
    fill(KM.ex_Mapobj.begin(), KM.ex_Mapobj.end(), 0);
    for (int i = 0; i < KM.love.size(); ++i) 
    {
        KM.ex_Curobj[i] = KM.love[i][0];
        for (int j = 1; j < KM.love[i].size(); ++j) 
        {
            KM.ex_Curobj[i] = max(KM.ex_Curobj[i], KM.love[i][j]);
        }
    }

    for (int i = 0; i < KM.love.size(); ++i) {

        fill(KM.slack.begin(), KM.slack.end(), 100);   

        int count = 0;
        while (1) 
        {
            fill(KM.vis_Curobj.begin(), KM.vis_Curobj.begin(), false);
            fill(KM.vis_Mapobj.begin(), KM.vis_Mapobj.begin(), false);


            if (dfs(i, KM, count)) break;  
            count++;
            int d = 1;
            for (int j = 0; j < KM.love.size(); ++j)
                if (!KM.vis_Mapobj[j]) d = min(d, KM.slack[j]);

            for (int j = 0; j < KM.love.size(); ++j) 
            {
                if (KM.vis_Curobj[j]) 
                {                
                    KM.ex_Curobj[j] -= d;
                    cout<<"Curobj expect :"<<KM.ex_Curobj[j]<<endl;
                    if(KM.ex_Curobj[j] < 0)break;
                }

                if (KM.vis_Mapobj[j]) 
                {
                    KM.ex_Mapobj[j] += d;
                    cout<<"Mapobj expect :"<<KM.ex_Mapobj[j]<<endl;
                }
                else 
                {
                    KM.slack[j] -= d;
                    cout<<"Mapobj slack :"<<KM.slack[j]<<endl;
                }
            }

        }
    }

}

//did not use, for test
void Object::ComputeRotation(Object::Object_3D& Object, Frame currentFrame, vector<cv::Vec4f> lines)
{

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



    vector<cv::Vec4f> lines_inmask;


    for(int i = 0; i < lines.size(); i++)
    {
        cv::Point point1(lines[i][0], lines[i][1]);  
        cv::Point point2(lines[i][2], lines[i][3]);  
        if((cv::pointPolygonTest(Object.mask, point1, false) >= 0) && (cv::pointPolygonTest(Object.mask, point2, false) >= 0))
        {
            lines_inmask.push_back(lines[i]);
        }
    }


    double minabgle = 200;
    double minabgleX = 200;
    double minabgleZ = 200;

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
 

}

//did not use, for test
double CalculateAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& q1, const cv::Point& q2)
{
    cv::Point vec1 = p2 - p1;  
    cv::Point vec2 = q2 - q1;

    double length1 = cv::norm(vec1);
    double length2 = cv::norm(vec2);

    double dotProduct = vec1.dot(vec2);
    double crossProduct = vec1.x * vec2.y - vec1.y * vec2.x;  // 叉积计算

    double angleRad = std::acos(dotProduct / (length1 * length2));

    if (crossProduct < 0)
        angleRad = -angleRad;

    double angleDeg = angleRad * (180.0 / CV_PI);

    return angleDeg;
}


//did not use, for test
bool areLinesParallel(const cv::Vec4f& line1, const cv::Vec4f& line2) {

    double slope1 = (line1[3] - line1[1]) / (line1[2] - line1[0]);

    double slope2 = (line2[3] - line2[1]) / (line2[2] - line2[0]);

    double epsilon = 0.01;

    if (std::abs(slope1 - slope2) < epsilon) {
        return true;
    }

    return false;
}

//did not use, for test
double calculateSlope(const cv::Vec4f& line) 
{
    double dx = line[2] - line[0];
    double dy = line[3] - line[1];
    if (std::abs(dx) < 1e-6) {
        return std::numeric_limits<double>::infinity();
    }
    return dy / dx;
}


 

}
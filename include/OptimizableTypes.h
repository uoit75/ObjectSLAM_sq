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

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/sim3.h>

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>
#include <iostream>



namespace ORB_SLAM3
{
// 左目纯位姿优化的边，左目点的重投影误差相对于左目位姿
class EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera *pCamera;
};

// 两个相机中的右目上的重投影误差与左目位姿的边
class EdgeSE3ProjectXYZOnlyPoseToBody : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera *pCamera;

    g2o::SE3Quat mTrl;
};


// 左目纯位姿优化的边，左目点的重投影误差相对于左目位姿以及三维点
class EdgeSE3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
    }

    virtual void linearizeOplus();

    GeometricCamera *pCamera;
};

// 两个相机中的右目上的重投影误差与左目位姿以及三维点的边
class EdgeSE3ProjectXYZToBody : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody();

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
    }

    virtual void linearizeOplus();

    GeometricCamera *pCamera;
    g2o::SE3Quat mTrl;
};

// sim3节点
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    // 原始值
    virtual void setToOriginImpl()
    {
        _estimate = g2o::Sim3();
    }

    // 更新
    virtual void oplusImpl(const double *update_)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double *>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s * estimate());
    }

    GeometricCamera *pCamera1, *pCamera2;

    bool _fix_scale;
};

// sim3边
class EdgeSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
    }

    // 自动求导，没错g2o也有自动求导
    // virtual void linearizeOplus();
};

// sim3反投的边
class EdgeInverseSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs - v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
    }

    // 自动求导，没错g2o也有自动求导
    // virtual void linearizeOplus();
};


//[]优化物体
// 顶点模型，模板参数：优化变量维度和数据类型  继承
class ObjectFittingVertex : public g2o::BaseVertex<4, Eigen::Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重置
  virtual void setToOriginImpl() override {
    _estimate << 0, 0, 0, 0;
  }


  // 更新
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector4d(update);
    // if(_estimate(0) < 0)_estimate(0) = _estimate(0) * -1;
    // if(_estimate(1) < 0)_estimate(1) = _estimate(1) * -1;
    // if(_estimate(2) < 0)_estimate(2) = _estimate(2) * -1;
    // applyBounds();
  }

    //   // 设置上下界的函数
    // void setBounds(const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) 
    // {
    //     _lowerBound = lower;
    //     _upperBound = upper;
    // }

    // 存盘和读盘：留空
  virtual bool read(std::istream &in) {}

  virtual bool write(std::ostream &out) const {}



// protected:
//     Eigen::Vector4d _lowerBound;
//     Eigen::Vector4d _upperBound;

//     // 应用上下界的函数
//     void applyBounds() {
//         _estimate = _estimate.cwiseMax(_lowerBound).cwiseMin(_upperBound);
//     }

  

};


class ObjectFittingVertexE12 : public g2o::BaseVertex<2, Eigen::Vector2d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重置
  virtual void setToOriginImpl() override {
    _estimate << 0, 0;
  }


  // 更新
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector2d(update);





    // if(_estimate(0) < 0)_estimate(0) = _estimate(0) * -1;
    // if(_estimate(1) < 0)_estimate(1) = _estimate(1) * -1;
    // if(_estimate(2) < 0)_estimate(2) = _estimate(2) * -1;
    // applyBounds();
  }

    //   // 设置上下界的函数
    // void setBounds(const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) 
    // {
    //     _lowerBound = lower;
    //     _upperBound = upper;
    // }

    // 存盘和读盘：留空
  virtual bool read(std::istream &in) {}

  virtual bool write(std::ostream &out) const {}



// protected:
//     Eigen::Vector4d _lowerBound;
//     Eigen::Vector4d _upperBound;

//     // 应用上下界的函数
//     void applyBounds() {
//         _estimate = _estimate.cwiseMax(_lowerBound).cwiseMin(_upperBound);
//     }

  

};





class ObjectFittingEdge: public g2o::BaseUnaryEdge<1,double,ObjectFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectFittingEdge(Eigen::Vector3f pos, Eigen::Vector3f center, double angleG, float e1, float e2) : BaseUnaryEdge(), _pos(pos), _center(center), _angleG(angleG), _e1(e1), _e2(e2) {}
    // 计算曲线模型误差
    void computeError()
    {
        const ObjectFittingVertex* v = static_cast<const ObjectFittingVertex*> (_vertices[0]);
        const Eigen::Vector4d XYZR = v->estimate();

        //计算旋转矩阵
        Eigen::Matrix4d Twq_R = Eigen::Matrix4d::Zero();

        double angleR = XYZR(3, 0);  

        // 填充旋转矩阵部分
        Twq_R(0, 0) = cos(angleR);
        Twq_R(0, 2) = sin(angleR);
        Twq_R(2, 0) = -sin(angleR);
        Twq_R(2, 2) = cos(angleR);
        Twq_R(1, 1) = 1.0;
        Twq_R(3, 3) = 1.0;

        //计算变换矩阵
        Eigen::Matrix4d Twq = Eigen::Matrix4d::Zero();
        Twq(1, 1) = cos(_angleG);
        Twq(1, 2) = -sin(_angleG);
        Twq(2, 1) = sin(_angleG);
        Twq(2, 2) = cos(_angleG);
        Twq(0, 0) = 1.0;
        Twq(3, 3) = 1.0;
        Twq(0, 3) = _center(0);
        Twq(1, 3) = _center(1);
        Twq(2, 3) = _center(2);


        Twq = Twq * Twq_R;

        Eigen::Matrix4d Twq_inv = Twq.inverse();


        // std::cout<<"_Twq_inv :"<<_Twq_inv<<std::endl;
        // std::cout<<"Twq_R :"<<Twq_R<<std::endl;


        // _Twq_inv = (_Twq * Twq_R).inv();

        // std::cout<<"after _Twq_inv :"<<_Twq_inv<<std::endl;

        // double euation1 = (_Twq_inv.at<float>(0,0)*_pos(0) + _Twq_inv.at<float>(0,1)*_pos(1) + _Twq_inv.at<float>(0,2)*_pos(2) + _Twq_inv.at<float>(0,3)) / (2*XYZ(0,0));
        // double euation2 = (_Twq_inv.at<float>(1,0)*_pos(0) + _Twq_inv.at<float>(1,1)*_pos(1) + _Twq_inv.at<float>(1,2)*_pos(2) + _Twq_inv.at<float>(1,3)) / (2*XYZ(1,0));
        // double euation3 = (_Twq_inv.at<float>(2,0)*_pos(0) + _Twq_inv.at<float>(2,1)*_pos(1) + _Twq_inv.at<float>(2,2)*_pos(2) + _Twq_inv.at<float>(2,3)) / (2*XYZ(2,0));
        double euation1 = (Twq_inv(0,0)*_pos(0) + Twq_inv(0,1)*_pos(1) + Twq_inv(0,2)*_pos(2) + Twq_inv(0,3)) / XYZR(0,0);
        double euation2 = (Twq_inv(1,0)*_pos(0) + Twq_inv(1,1)*_pos(1) + Twq_inv(1,2)*_pos(2) + Twq_inv(1,3)) / XYZR(1,0);
        double euation3 = (Twq_inv(2,0)*_pos(0) + Twq_inv(2,1)*_pos(1) + Twq_inv(2,2)*_pos(2) + Twq_inv(2,3)) / XYZR(2,0);
        double F = pow((pow(euation1, 2/_e2) + pow(euation2, 2/_e2)), _e2/_e1) + pow(euation3, 2/_e1);
        double beta = pow(F, -(_e1/2));
        double d = (_pos - _center).norm() * abs(1 - beta);
        double F_e1 = pow(F, _e1);

        // std::cout<<"F in g2o :"<<F<<std::endl;
        // std::cout<<"d in g2o :"<<d<<std::endl;
        // std::cout<<"norm in g2o :"<<(_pos - _center).norm()<<std::endl;
        // std::cout<<"f in g2o :"<<abs(1-pow(F, -(_e1/2)))<<std::endl;
        // std::cout<<"F in g2o :"<<F<<std::endl;
        // std::cout<<"euation1 in g2o :"<<euation1<<std::endl;
        // std::cout<<"euation2 in g2o :"<<euation2<<std::endl;
        // std::cout<<"euation3 in g2o :"<<euation3<<std::endl;
    
        // _error(0,0) = pow((1 - 1/beta), 2);
        // _error(0,0) = pow((sqrt(XYZ(0,0)*XYZ(1,0)*XYZ(2,0)) * (F_e1 - 1)), 2);
        // _error(0,0) = pow((F_e1 - 1), 2);
        _error(0,0) = abs(F_e1 - 1);
        // _error(0,0) = abs(1 - F);
        // _error(0,0) = d;
        // std::cout<<"error in g2o :"<<_error(0,0)<<std::endl;

    }
    virtual bool read(std::istream& in ) {}
    virtual bool write(std::ostream& out ) const {}
public:
    Eigen::Vector3f _pos;  
    Eigen::Vector3f _center;  
    double _angleG;
    float _e1;
    float _e2;
};


class ObjectFittingEdgeE12: public g2o::BaseUnaryEdge<1,double,ObjectFittingVertexE12>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectFittingEdgeE12(Eigen::Vector3f pos, Eigen::Vector3f center, Eigen::Vector3f xyz, double angleG, double angleR, float e1, float e2) : BaseUnaryEdge(), _pos(pos), _center(center), _xyz(xyz), _angleG(angleG), _angleR(angleR), _e1(e1), _e2(e2) {}
    // 计算曲线模型误差
    void computeError()
    {
        const ObjectFittingVertexE12* v = static_cast<const ObjectFittingVertexE12*> (_vertices[0]);
        Eigen::Vector2d XYZR = v->estimate();

        //计算旋转矩阵
        Eigen::Matrix4d Twq_R = Eigen::Matrix4d::Zero();

        // double angleR = XYZR(3, 0);  

        // 填充旋转矩阵部分
        Twq_R(0, 0) = cos(_angleR);
        Twq_R(0, 2) = sin(_angleR);
        Twq_R(2, 0) = -sin(_angleR);
        Twq_R(2, 2) = cos(_angleR);
        Twq_R(1, 1) = 1.0;
        Twq_R(3, 3) = 1.0;

        //计算变换矩阵
        Eigen::Matrix4d Twq = Eigen::Matrix4d::Zero();
        Twq(1, 1) = cos(_angleG);
        Twq(1, 2) = -sin(_angleG);
        Twq(2, 1) = sin(_angleG);
        Twq(2, 2) = cos(_angleG);
        Twq(0, 0) = 1.0;
        Twq(3, 3) = 1.0;
        Twq(0, 3) = _center(0);
        Twq(1, 3) = _center(1);
        Twq(2, 3) = _center(2);


        Twq = Twq * Twq_R;

        Eigen::Matrix4d Twq_inv = Twq.inverse();


        // std::cout<<"_Twq_inv :"<<_Twq_inv<<std::endl;
        // std::cout<<"Twq_R :"<<Twq_R<<std::endl;


        // _Twq_inv = (_Twq * Twq_R).inv();

        // std::cout<<"after _Twq_inv :"<<_Twq_inv<<std::endl;
        // std::cout<<"e1e2 in g2o :"<<XYZR(4,0)<<" "<<XYZR(5,0)<<std::endl;

        // if(XYZR(0,0) <= 0.1)XYZR(0,0) = 0.1;
        // else if(XYZR(0,0) >= 1.9)XYZR(0,0) = 1.9;
        // else XYZR(0,0) = 0.1;
        // if(XYZR(1,0) <= 0.1)XYZR(1,0) = 0.1;
        // else if(XYZR(1,0) >= 1.9)XYZR(1,0) = 1.9;
        // else XYZR(1,0) = 0.1;


        // double euation1 = (Twq_inv(0,0)*_pos(0) + Twq_inv(0,1)*_pos(1) + Twq_inv(0,2)*_pos(2) + Twq_inv(0,3)) / _xyz(0);
        // double euation2 = (Twq_inv(1,0)*_pos(0) + Twq_inv(1,1)*_pos(1) + Twq_inv(1,2)*_pos(2) + Twq_inv(1,3)) / _xyz(1);
        // double euation3 = (Twq_inv(2,0)*_pos(0) + Twq_inv(2,1)*_pos(1) + Twq_inv(2,2)*_pos(2) + Twq_inv(2,3)) / _xyz(2);
        // double F = pow((pow(euation1, 2/XYZR(1,0)) + pow(euation2, 2/XYZR(1,0))), XYZR(1,0)/XYZR(0,0)) + pow(euation3, 2/XYZR(0,0));
        // double beta = pow(F, -(XYZR(0,0)/2));
        // double d = (_pos - _center).norm() * abs(1 - beta);
        // double F_e1 = pow(F, XYZR(0,0));
        double euation1 = (Twq_inv(0,0)*_pos(0) + Twq_inv(0,1)*_pos(1) + Twq_inv(0,2)*_pos(2) + Twq_inv(0,3)) / _xyz(0);
        double euation2 = (Twq_inv(1,0)*_pos(0) + Twq_inv(1,1)*_pos(1) + Twq_inv(1,2)*_pos(2) + Twq_inv(1,3)) / _xyz(1);
        double euation3 = (Twq_inv(2,0)*_pos(0) + Twq_inv(2,1)*_pos(1) + Twq_inv(2,2)*_pos(2) + Twq_inv(2,3)) / _xyz(2);
        double F = pow((pow(abs(euation1), 2/XYZR(1,0)) + pow(abs(euation2), 2/XYZR(1,0))), XYZR(1,0)/XYZR(0,0)) + pow(abs(euation3), 2/XYZR(0,0));
        double beta = pow(F, -(XYZR(0,0)/2));
        double d = (_pos - _center).norm() * abs(1 - beta);
        double F_e1 = pow(F, XYZR(0,0));




        // std::cout<<"e1e2 in g2o :"<<XYZR(0,0)<<" "<<XYZR(1,0)<<std::endl;

        // std::cout<<"Fe1 in g2o :"<<F_e1<<std::endl;
        // std::cout<<"F in g2o :"<<F<<std::endl;
        // std::cout<<"d in g2o :"<<d<<std::endl;
        // std::cout<<"norm in g2o :"<<(_pos - _center).norm()<<std::endl;
        // std::cout<<"f in g2o :"<<abs(1-pow(F, -(_e1/2)))<<std::endl;
        // std::cout<<"F in g2o :"<<F<<std::endl;
        // std::cout<<"euation1 in g2o :"<<euation1<<std::endl;
        // std::cout<<"euation2 in g2o :"<<euation2<<std::endl;
        // std::cout<<"euation3 in g2o :"<<euation3<<std::endl;
    
        // _error(0,0) = pow((1 - 1/beta), 2);
        // _error(0,0) = pow((sqrt(XYZ(0,0)*XYZ(1,0)*XYZ(2,0)) * (F_e1 - 1)), 2);
        // _error(0,0) = pow((F_e1 - 1), 2);
        _error(0,0) = abs(F_e1 - 1);
        // _error(0,0) = abs(1 - F);
        // _error(0,0) = d;
        // std::cout<<"error in g2o :"<<_error(0,0)<<std::endl;

    }
    virtual bool read(std::istream& in ) {}
    virtual bool write(std::ostream& out ) const {}
public:
    Eigen::Vector3f _pos;  
    Eigen::Vector3f _center;  
    Eigen::Vector3f _xyz;  
    double _angleG;
    double _angleR;
    float _e1;
    float _e2;
};



class ObjectAnalyseEdge: public g2o::BaseUnaryEdge<1,double,ObjectFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectAnalyseEdge(Eigen::Vector3f pos, Eigen::Vector3f center, double angleG, float e1, float e2) : BaseUnaryEdge(), _pos(pos), _center(center), _angleG(angleG), _e1(e1), _e2(e2) {}
    // 计算曲线模型误差
    void computeError()
    {
        const ObjectFittingVertex* v = static_cast<const ObjectFittingVertex*> (_vertices[0]);
        const Eigen::Vector4d XYZR = v->estimate();

        //计算旋转矩阵
        Eigen::Matrix4d Twq_R = Eigen::Matrix4d::Zero();

        double angleR = XYZR(3, 0);  

        // 填充旋转矩阵部分
        Twq_R(0, 0) = cos(angleR);
        Twq_R(0, 2) = sin(angleR);
        Twq_R(2, 0) = -sin(angleR);
        Twq_R(2, 2) = cos(angleR);
        Twq_R(1, 1) = 1.0;
        Twq_R(3, 3) = 1.0;

        //计算变换矩阵
        Eigen::Matrix4d Twq = Eigen::Matrix4d::Zero();
        Twq(1, 1) = cos(_angleG);
        Twq(1, 2) = -sin(_angleG);
        Twq(2, 1) = sin(_angleG);
        Twq(2, 2) = cos(_angleG);
        Twq(0, 0) = 1.0;
        Twq(3, 3) = 1.0;
        Twq(0, 3) = _center(0);
        Twq(1, 3) = _center(1);
        Twq(2, 3) = _center(2);


        Twq = Twq * Twq_R;

        Eigen::Matrix4d Twq_inv = Twq.inverse();


        // std::cout<<"_Twq_inv :"<<_Twq_inv<<std::endl;
        // std::cout<<"Twq_R :"<<Twq_R<<std::endl;


        // _Twq_inv = (_Twq * Twq_R).inv();

        // std::cout<<"after _Twq_inv :"<<_Twq_inv<<std::endl;

        // double euation1 = (_Twq_inv.at<float>(0,0)*_pos(0) + _Twq_inv.at<float>(0,1)*_pos(1) + _Twq_inv.at<float>(0,2)*_pos(2) + _Twq_inv.at<float>(0,3)) / (2*XYZ(0,0));
        // double euation2 = (_Twq_inv.at<float>(1,0)*_pos(0) + _Twq_inv.at<float>(1,1)*_pos(1) + _Twq_inv.at<float>(1,2)*_pos(2) + _Twq_inv.at<float>(1,3)) / (2*XYZ(1,0));
        // double euation3 = (_Twq_inv.at<float>(2,0)*_pos(0) + _Twq_inv.at<float>(2,1)*_pos(1) + _Twq_inv.at<float>(2,2)*_pos(2) + _Twq_inv.at<float>(2,3)) / (2*XYZ(2,0));
        double euation1 = (Twq_inv(0,0)*_pos(0) + Twq_inv(0,1)*_pos(1) + Twq_inv(0,2)*_pos(2) + Twq_inv(0,3)) / XYZR(0,0);
        double euation2 = (Twq_inv(1,0)*_pos(0) + Twq_inv(1,1)*_pos(1) + Twq_inv(1,2)*_pos(2) + Twq_inv(1,3)) / XYZR(1,0);
        double euation3 = (Twq_inv(2,0)*_pos(0) + Twq_inv(2,1)*_pos(1) + Twq_inv(2,2)*_pos(2) + Twq_inv(2,3)) / XYZR(2,0);
        double F = pow((pow(euation1, 2/_e2) + pow(euation2, 2/_e2)), _e2/_e1) + pow(euation3, 2/_e1);
        double beta = pow(F, -(_e1/2));
        double d = (_pos - _center).norm() * abs(1 - beta);
        double F_e1 = pow(F, _e1);

        // std::cout<<"F in g2o :"<<F<<std::endl;
        // std::cout<<"d in g2o :"<<d<<std::endl;
        // std::cout<<"norm in g2o :"<<(_pos - _center).norm()<<std::endl;
        // std::cout<<"f in g2o :"<<abs(1-pow(F, -(_e1/2)))<<std::endl;
        // std::cout<<"F in g2o :"<<F<<std::endl;
        // std::cout<<"euation1 in g2o :"<<euation1<<std::endl;
        // std::cout<<"euation2 in g2o :"<<euation2<<std::endl;
        // std::cout<<"euation3 in g2o :"<<euation3<<std::endl;
    
        // _error(0,0) = pow((1 - 1/beta), 2);
        // _error(0,0) = pow((sqrt(XYZ(0,0)*XYZ(1,0)*XYZ(2,0)) * (F_e1 - 1)), 2);
        // _error(0,0) = pow((F_e1 - 1), 2);
        _error(0,0) = abs(F_e1 - 1);
        // _error(0,0) = abs(1 - F);
        // _error(0,0) = d;
        // std::cout<<"error in g2o :"<<_error(0,0)<<std::endl;

    }
    virtual bool read(std::istream& in ) {}
    virtual bool write(std::ostream& out ) const {}
public:
    Eigen::Vector3f _pos;  
    Eigen::Vector3f _center;  
    double _angleG;
    float _e1;
    float _e2;
};




//[]优化角度
// 顶点模型，模板参数：优化变量维度和数据类型  继承
class ObjectRotationVertex : public g2o::BaseVertex<1, Eigen::VectorXd> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ObjectRotationVertex() {}

  // 重置
  virtual void setToOriginImpl() override {
    _estimate = Eigen::VectorXd::Zero(1);
  }

  // 更新
  virtual void oplusImpl(const double *update) override {
    _estimate[0] += update[0];
  }

  // 存盘和读盘：留空
  virtual bool read(std::istream &in) {}

  virtual bool write(std::ostream &out) const {}



};





}

#endif // ORB_SLAM3_OPTIMIZABLETYPES_H

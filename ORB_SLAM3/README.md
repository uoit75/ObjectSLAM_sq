# VSLAM_voxelized

#### 介绍
这是一个修改版本的视觉定位系统，发布了视觉定位话题和15x15的协方差矩阵，用于导航和规控来实现路劲规划，此版本还可以和GNSS/RTK融合用于室外的视觉导航(中低速情况)，视觉建图为特征点的体素化。 室内误差±3cm/m.

#### 软件架构
软件架构说明


#### 安装教程



1.  需求系统：Ubuntu20.04  备注：【如若在Ubuntu18.04上使用时需要更改Opencv的版本(<4.0版本)，同时需要修改src/xx目录下的部分代码】编译时会报错，需要根据需求去修改
2.  需要第三方库： 
               Opencv4.x, Eigen3.4, Opencv_comtrib, Panglion0.6, Ceres-solver.
3.  编译方式举例：
                `cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_GENERATE_PKGCONFIG=YES -DCMAKE_INSTALL_PREFIX=/usr/local/opencv4.7.0  -D OPENCV_EXTRA_MODULES_PATH=/home/kalku/Data/ORB_SLAM/opencv_contrib/modules  -DCMAKE_C_FLAGS_RELEASE="-march=native -Ofast -funroll-loops -fopenmp -flto" -DCMAKE_CXX_FLAGS_RELEASE="-march=native -Ofast -funroll-loops -fopenmp -flto" -DWITH_TBB=ON -DWITH_OPENMP=ON ..`  哪找此类方式，适合Arm（Nvidia产品，Rock产品）和X86架构，都可以部署。
4.  视觉展示效果：![稀疏体素化效果](error/WechatIMG77.jpeg)
                ![双目鱼眼效果](WechatIMG76.jpeg)
                ![体素化计算](error/WechatIMG75.jpeg)
                
    

#### 使用说明

1.  待补充
2.  待补充
3.  待补充

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)

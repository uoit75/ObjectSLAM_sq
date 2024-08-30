# VSLAM_voxelized

#### Description
This is the system that publishes a visual localization topic and a 15x15 covariance matrix for navigation and control to achieve path planning. The visual mapping is based on voxelized feature points

#### Software Architecture
Software architecture description

#### Installation

1.Required System: Ubuntu 20.04
Note: If you're using Ubuntu 18.04, you'll need to change the version of OpenCV to a version less than 4.0. You will also need to modify some code in the src/xx directory. Compilation errors will occur, so modifications are needed based on your requirements.

2.Required Third-party Libraries:
    Opencv 4.x, Eigen 3.4, Opencv_contrib, Pangolin 0.6, Ceres-solver.

3.Example Compilation Method:
`cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_GENERATE_PKGCONFIG=YES -DCMAKE_INSTALL_PREFIX=/usr/local/opencv4.7.0 -D OPENCV_EXTRA_MODULES_PATH=/home/kalku/Data/ORB_SLAM/opencv_contrib/modules -DCMAKE_C_FLAGS_RELEASE="-march=native -Ofast -funroll-loops -fopenmp -flto" -DCMAKE_CXX_FLAGS_RELEASE="-march=native -Ofast -funroll-loops -fopenmp -flto" -DWITH_TBB=ON -DWITH_OPENMP=ON ..`
This method is suitable for both ARM (Nvidia products, Rock products) and x86 architectures and can be deployed on both.

#### Instructions

1.  xxxx
2.  xxxx
3.  xxxx

#### Contribution

1.  Fork the repository
2.  Create Feat_xxx branch
3.  Commit your code
4.  Create Pull Request


#### Gitee Feature

1.  You can use Readme\_XXX.md to support different languages, such as Readme\_en.md, Readme\_zh.md
2.  Gitee blog [blog.gitee.com](https://blog.gitee.com)
3.  Explore open source project [https://gitee.com/explore](https://gitee.com/explore)
4.  The most valuable open source project [GVP](https://gitee.com/gvp)
5.  The manual of Gitee [https://gitee.com/help](https://gitee.com/help)
6.  The most popular members  [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)

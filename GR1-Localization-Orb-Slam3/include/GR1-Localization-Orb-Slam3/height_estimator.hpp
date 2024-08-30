#ifndef __HEIGHT_EST_HPP__
#define __HEIGHT_EST_HPP__

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <cmath>
#include <Eigen/Core>
using namespace std;

struct GridCell {
    int x; // 格子坐标
    int y; // 格子坐标
    float z; // 使用z来表示高度
};

class HeightEstimator
{
public:
    HeightEstimator();
    ~HeightEstimator();
    void processDepth(const cv::Mat &depthImage, Eigen::Matrix2d &heightMap);

private:
    int height, width, cellHeight, cellWidth;
    int gridHeight, gridWidth;
    float cameraHeight, maxRange;
    float cx, cy, fx, fy;

    void depthToWorld(int x, int y, float depthValue, GridCell& cell);
    void processGridCells(const float *depthImage, vector<vector<GridCell>> &grid, Eigen::Matrix2d &heightMap);

    void printGridData(const vector<vector<GridCell>> &grid);
};

#endif
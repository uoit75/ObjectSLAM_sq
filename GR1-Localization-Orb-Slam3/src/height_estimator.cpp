#include "height_estimator.hpp"

HeightEstimator::HeightEstimator() : 
height(480), 
width(640),
gridHeight(10), 
gridWidth(10), 
cellHeight(height / gridHeight), 
cellWidth(width / gridWidth), 
cameraHeight(0.5), 
maxRange(2.0), 
cx(width / 2), 
cy(height / 2), 
fx(525.0), 
fy(525.0)
{ };

HeightEstimator::~HeightEstimator() { };

void HeightEstimator::depthToWorld(int x, int y, float depthValue, GridCell& cell)
{
    float cameraX = (x - cx) * depthValue / fx;
    float cameraY = (y - cy) * depthValue / fy;

    //?
    cell.x = static_cast<int>((cameraX / 640.0) * gridWidth);
    cell.y = static_cast<int>((cameraY / 360.0) * gridHeight);
    cell.z = cameraHeight - cameraY;
}

void HeightEstimator::processGridCells(const float *depthImage, vector<vector<GridCell>> &grid, Eigen::Matrix2d &heightMap) {
    for (int gridY = 0; gridY < gridHeight; gridY++) {
        for (int gridX = 0; gridX < gridWidth; gridX++) {
            vector<float> heights;
            for (int y = gridY * cellHeight; y < (gridY + 1) * cellHeight; y++) {
                for (int x = gridX * cellWidth; x < (gridX + 1) * cellWidth; x++) {
                    int index = x + y * width;
                    float depth = depthImage[index];
                    if (depth != INFINITY && depth != -INFINITY && !isnan(depth)) {
                        depthToWorld(x, y, depth, grid[gridY][gridX]);
                        heights.push_back(grid[gridY][gridX].z);
                    }
                }
            }
            if (!heights.empty()) {
                sort(heights.begin(), heights.end());
                size_t n = heights.size();
                grid[gridY][gridX].z = (n % 2 == 0) ? (heights[n / 2 - 1] + heights[n / 2]) / 2 : heights[n / 2];
            } else {
                grid[gridY][gridX].z = 0.0;
            }
            heightMap(gridY, gridX) = grid[gridY][gridX].z;
        }
    }
}

void HeightEstimator::printGridData(const vector<vector<GridCell>> &grid) {
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            cout << fixed << setprecision(3);
            cout << "(" << x << ", " << y << ", " << grid[y][x].z << ") ";
        }
        cout << endl;
    }
    cout << "---------------------------------" << endl;
}

void HeightEstimator::processDepth(const cv::Mat &depthRaw, Eigen::Matrix2d &heightMap) {
    float* depthImage = new float[width * height];
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            depthImage[i * width + j] = depthRaw.at<float>(i, j);
        }
    }
    vector<vector<GridCell>> grid(gridHeight, vector<GridCell>(gridWidth));
    heightMap = Eigen::Matrix2d::Zero(gridHeight, gridWidth);
    processGridCells(depthImage, grid, heightMap);
    printGridData(grid);
    
    
}
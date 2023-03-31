#ifndef TYPE_H_
#define TYPE_H_

#include <vector>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

namespace kitti
{
struct Label
{
  std::string type;
  double truncated;
  double occluded;
  double alpha;
  double left, top, right, bottom;  // 2D bbox
  double height, width, length;
  double loc_x, loc_y, loc_z;  // camera coordinates
  double rot_y;
  double score;
};

struct Calibration
{
  Eigen::Matrix<double, 3, 4> P0, P1, P2, P3;
  Eigen::Matrix<double, 3, 3> R0_rect;
  Eigen::Matrix<double, 3, 4> velo_to_cam;
  Eigen::Matrix<double, 3, 4> imu_to_velo;
};

struct Data
{
  Calibration calibration;
  cv::Mat image;
  std::vector<kitti::Label> labels;
};
}  // namespace kitti

struct Attributes
{
  double loc_x, loc_y, loc_z;
  double height, width, length;
  double rot_y;
};

namespace background
{
constexpr int16_t HEIGHT = 600;
constexpr int16_t WIDTH = 600;
constexpr int16_t HALF_WIDTH = 300;
constexpr int16_t BOX_SCALE = 15;
}

constexpr int8_t COLOR_MAP_SIZE = 100;

#endif  // TYPE_H_

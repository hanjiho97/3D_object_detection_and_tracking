#ifndef TYPE_H_
#define TYPE_H_

#include <cstdint>
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
  uint8_t state;
  double loc_x, loc_y, loc_z;
  double height, width, length;
  double rot_y;
};

#endif  // TYPE_H_

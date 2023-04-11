#ifndef PROJECTION_H
#define PROJECTION_H

#include <cmath>
#include <cstdint>
#include <vector>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "result_viewer/Type.h"

class Projection
{
public:
  Projection();
  virtual ~Projection();
  std::vector<cv::Point> get_2D_corners(
    const Attributes& attributes,
    const Eigen::Matrix<double, 3, 4>& P2);
  std::vector<cv::Point> get_topview_conrers(
    uint16_t background_half_width,
    uint16_t background_height,
    uint8_t background_box_scale);

private:
  Eigen::Matrix<double, 3, 14> corners1_3D_;
  Eigen::Matrix<double, 3, 14> corners2_3D_;
  Eigen::Matrix<double, 3, 14> corners1_2D_;
  Eigen::Matrix<double, 3, 14> corners2_2D_;

  void set_3D_corners(
    Eigen::Matrix<double, 3, 14>& corners1_3D,
    Eigen::Matrix<double, 3, 14>& corners2_3D,
    const Attributes& attributes);
  bool rotate_3D_corners(
    Eigen::Matrix<double, 3, 14>& corners_3D,
    const double rot_y);
  bool move_3D_corners_to_world(
    Eigen::Matrix<double, 3, 14>& corners_3D,
    const Attributes& attributes);
  bool project_3D_to_2D(
    const Eigen::Matrix<double, 3, 14>& corners_3D,
    const Eigen::Matrix<double, 3, 4>& P2,
    Eigen::Matrix<double, 3, 14>& corners_2D);
};

#endif  //
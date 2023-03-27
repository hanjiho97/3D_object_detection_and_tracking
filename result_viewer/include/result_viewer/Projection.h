#ifndef PROJECTION_H
#define PROJECTION_H

#include <iostream>
#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "result_viewer/Type.h"

class Projection
{
public:
  Projection();
  virtual ~Projection();
  void read_data(
    const Attributes& attributes, 
    const Eigen::Matrix<double, 3, 4> P2);
  std::vector<cv::Point> get_2D_corners();

private:
  Eigen::Matrix<double, 3, 8> corners_3D_;
  Eigen::Matrix<double, 3, 8> corners_2D_;
  Eigen::Matrix<double, 3, 4> P2_;
  Attributes attributes_;

  void set_3D_corners( 
    Eigen::Matrix<double, 3, 8>& corners_3D,
    const Attributes& attributes);
  bool rotate_3D_corners(
    Eigen::Matrix<double, 3, 8>& corners_3D,
    const double rot_y);
  bool move_3D_corners_to_world(
    Eigen::Matrix<double, 3, 8>& corners_3D,
    const Attributes& attributes);
  bool project_3D_to_2D(
    const Eigen::Matrix<double, 3, 8>& corners_3D,
    const Eigen::Matrix<double, 3, 4>& P2,
    Eigen::Matrix<double, 3, 8>& corners_2D);
};

#endif  //
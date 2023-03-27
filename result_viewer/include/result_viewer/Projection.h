#ifndef PROJECTION_H
#define PROJECTION_H

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
  bool rotate_3D_corners();
  bool move_3D_corners_to_world();
  bool move_velo_to_cam();
  bool project_3D_to_2D(
    const Eigen::Matrix<double, 3, 8>& corners_3D,
    Eigen::Matrix<double, 3, 8>& corners_2D,
    );
  bool read_data(Attributes attributes_, Eigen::VertorXd x_);
private:
  Eigen::Matrix<double, 3, 8> corners_3D;
  Eigen::Matrix<double, 3, 8> corners_2D;
  Eigen::VertorXd x_;
  Attributes attributes_;
};

#endif  //
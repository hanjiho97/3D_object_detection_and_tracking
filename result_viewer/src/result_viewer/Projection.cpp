#include "result_viewer/Projection.h"

Projection::Projection() {}

Projection::~Projection() {}

void Projection::set_3D_corners( 
    Eigen::Matrix<double, 3, 8>& corners_3D,
    const Attributes& attributes)
{
  return;
}

bool Projection::rotate_3D_corners(
  Eigen::Matrix<double, 3, 8>& corners_3D,
  const double rot_y)
{
  return 0;
}

bool Projection::move_3D_corners_to_world(
Eigen::Matrix<double, 3, 8>& corners_3D,
const Attributes& attributes)
{
  return 0;
}

bool Projection::project_3D_to_2D(
  const Eigen::Matrix<double, 3, 8>& corners_3D,
  const Eigen::Matrix<double, 3, 4>& P2,
  Eigen::Matrix<double, 3, 8>& corners_2D)
{
  return 0;
}

void Projection::read_data(
  const Attributes& attributes, 
  const Eigen::Matrix<double, 3, 4> P2)
{
  attributes_ = attributes;
  P2_ = P2;
  return;
}

std::vector<cv::Point> Projection::get_2D_corners()
{
  std::vector<cv::Point> Points;
  set_3D_corners(corners_3D_, attributes_);
  rotate_3D_corners(corners_3D_, attributes_.rot_y);
  move_3D_corners_to_world(corners_3D_, attributes_);
  project_3D_to_2D(corners_3D_, P2_, corners_2D_);
  return Points;
}
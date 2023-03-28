#include "result_viewer/Projection.h"

Projection::Projection() {}

Projection::~Projection() {}

void Projection::set_3D_corners(
  Eigen::Matrix<double, 3, 14>& corners1_3D,
  Eigen::Matrix<double, 3, 14>& corners2_3D,
  const Attributes& attributes)
{
  double height = attributes.height;
  double width = attributes.width;
  double length = attributes.length;
  corners1_3D.row(0) << -length / 2, -length / 2, -length / 2, -length / 2,
    -length / 2, -length / 2, length / 2, length / 2, -length / 2, length / 2,
    length / 2, -length / 2, length / 2, length / 2;
  corners1_3D.row(1) << -width, -width, 0, 0, 0, -width, -width, 0, -width,
    -width, -width, -width, -width, 0;
  corners1_3D.row(2) << -height / 2, height / 2, height / 2, -height / 2,
    height / 2, height / 2, height / 2, height / 2, -height / 2, -height / 2,
    height / 2, height / 2, height / 2, height / 2;

  corners2_3D.row(0) << length / 2, length / 2, length / 2, length / 2,
    -length / 2, -length / 2, length / 2, length / 2, -length / 2, length / 2,
    length / 2, -length / 2, length / 2, length / 2;
  corners2_3D.row(1) << -width, -width, 0, 0, 0, -width, -width, 0, 0, 0, 0, 0,
    0, -width;
  corners2_3D.row(2) << -height / 2, height / 2, height / 2, -height / 2,
    -height / 2, -height / 2, -height / 2, -height / 2, -height / 2,
    -height / 2, height / 2, height / 2, -height / 2, -height / 2;
  return;
}

bool Projection::rotate_3D_corners(
  Eigen::Matrix<double, 3, 14>& corners_3D,
  const double rot_y)
{
  Eigen::Matrix<double, 3, 3> rotation_matrix;
  rotation_matrix.row(0) << std::cos(rot_y), 0, std::sin(rot_y);
  rotation_matrix.row(1) << 0, 1, 0;
  rotation_matrix.row(2) << -std::sin(rot_y), 0, std::cos(rot_y);
  corners_3D = rotation_matrix * corners_3D;
  return 0;
}

bool Projection::move_3D_corners_to_world(
  Eigen::Matrix<double, 3, 14>& corners_3D,
  const Attributes& attributes)
{
  Eigen::Vector3d xyz;
  xyz << attributes.loc_x, attributes.loc_y, attributes.loc_z;
  for (uint8_t col = 0; col < 14; ++col)
  {
    corners_3D.col(col) += xyz;
  }
  return 0;
}

bool Projection::project_3D_to_2D(
  const Eigen::Matrix<double, 3, 14>& corners_3D,
  const Eigen::Matrix<double, 3, 4>& P2,
  Eigen::Matrix<double, 3, 14>& corners_2D)
{
  Eigen::Matrix<double, 4, 14> corners =
    Eigen::Matrix<double, 4, 14>::Constant(1);
  corners.block(0, 0, 3, 14) << corners_3D;
  corners_2D = P2 * corners;
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
  std::vector<cv::Point> points;
  cv::Point point;
  set_3D_corners(corners1_3D_, corners2_3D_, attributes_);
  rotate_3D_corners(corners1_3D_, attributes_.rot_y);
  rotate_3D_corners(corners2_3D_, attributes_.rot_y);
  move_3D_corners_to_world(corners1_3D_, attributes_);
  move_3D_corners_to_world(corners2_3D_, attributes_);
  project_3D_to_2D(corners1_3D_, P2_, corners1_2D_);
  project_3D_to_2D(corners2_3D_, P2_, corners2_2D_);
  for (uint8_t col = 0; col < 14; ++col)
  {
    point.x =
      static_cast<int16_t>(corners1_2D_(0, col) / corners1_2D_(2, col));
    point.y =
      static_cast<int16_t>(corners1_2D_(1, col) / corners1_2D_(2, col));
    points.push_back(point);

    point.x =
      static_cast<int16_t>(corners2_2D_(0, col) / corners2_2D_(2, col));
    point.y =
      static_cast<int16_t>(corners2_2D_(1, col) / corners2_2D_(2, col));
    points.push_back(point);
  }
  return points;
}
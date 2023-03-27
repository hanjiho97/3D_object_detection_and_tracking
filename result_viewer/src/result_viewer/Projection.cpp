#include "result_viewer/Projection.h"

Projection::Projection() {}

Projection::~Projection() {}

void Projection::set_3D_corners( 
    Eigen::Matrix<double, 3, 8>& corners_3D,
    const Attributes& attributes)
{
  double height = attributes.height;
  double width = attributes.width;
  double length = attributes.length;
  corners_3D.row(0) << -length/2, length/2, length/2, length/2, 
                        length/2, -length/2, -length/2, -length/2;
  corners_3D.row(1) << -width, -width, 0, 0, 
                       -width, -width, 0, 0;
  corners_3D.row(2) << -height/2, -height/2, -height/2, height/2, 
                        height/2, height/2, height/2, -height/2;
  return;
}

bool Projection::rotate_3D_corners(
  Eigen::Matrix<double, 3, 8>& corners_3D,
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
Eigen::Matrix<double, 3, 8>& corners_3D,
const Attributes& attributes)
{
  Eigen::Vector3d xyz;
  xyz << attributes.loc_x, attributes.loc_y, attributes.loc_z;
  for(uint8_t col=0; col<8; ++col)
  {
    corners_3D.col(col) += xyz;
  }
  return 0;
}

bool Projection::project_3D_to_2D(
  const Eigen::Matrix<double, 3, 8>& corners_3D,
  const Eigen::Matrix<double, 3, 4>& P2,
  Eigen::Matrix<double, 3, 8>& corners_2D)
{
  Eigen::Matrix<double, 4, 8> corners = Eigen::Matrix<double, 4, 8>::Constant(1);
  corners.block(0, 0, 3, 8) << corners_3D;
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
  set_3D_corners(corners_3D_, attributes_);
  rotate_3D_corners(corners_3D_, attributes_.rot_y);
  move_3D_corners_to_world(corners_3D_, attributes_);
  project_3D_to_2D(corners_3D_, P2_, corners_2D_);
  for (uint8_t col=0; col<8; ++col)
  {
    point.x = static_cast<uint16_t>(corners_2D_(0, col) / corners_2D_(2, col));
    point.y = static_cast<uint16_t>(corners_2D_(1, col) / corners_2D_(2, col));
    points.push_back(point);
  }
  std::cout << points[7] << std::endl;
  return points;
}
#include "object_tracking/Tracker.h"


Track::Track(const Measurement& meas, uint id)
{
  id_ = id;

  Eigen::VectorXd z = meas.get_z();
  Eigen::VectorXd pos_cam = Eigen::VectorXd::Ones(4);
  pos_cam.block<3, 1>(0, 0) = z;
  Eigen::MatrixXd cam_to_veh = meas.get_cam_to_veh();
  Eigen::VectorXd pos_veh = cam_to_veh * pos_cam;

  rot_cam_to_veh_ = cam_to_veh.block<3, 3>(0, 0);

  x_ = Eigen::VectorXd::Zero(6);
  x_.block<3, 1>(0, 0) = pos_veh.block<3, 1>(0, 0);

  P_ = Eigen::MatrixXd::Zero(6, 6);
  P_.block<3, 3>(0, 0) =
    rot_cam_to_veh_ * meas.get_R() * rot_cam_to_veh_.transpose();

  double sigma_p44 = 50.0;
  double sigma_p55 = 50.0;
  double sigma_p66 = 5.0;

  P_(3, 3) = sigma_p44 * sigma_p44;
  P_(4, 4) = sigma_p55 * sigma_p55;
  P_(5, 5) = sigma_p66 * sigma_p66;

  t_ = meas.get_t();
  attributes_ = meas.get_attributes();
  double meas_rot_y = attributes_.rot_y;
  attributes_.rot_y = std::acos(
    rot_cam_to_veh_(0, 0) * std::cos(meas_rot_y) +
    rot_cam_to_veh_(0, 1) * std::sin(meas_rot_y));

  state_ = 0;
  score_ = 1.0 / 6.0;
}

Track::~Track() {}

Eigen::VectorXd Track::get_x()
{
  return x_;
}
Eigen::MatrixXd Track::get_P()
{
  return P_;
}
double Track::get_t()
{
  return t_;
}

void Track::update_attributes(const Measurement& meas)
{
  t_ = meas.get_t();
  Attributes meas_attributes = meas.get_attributes();
  attributes_.height = 0.9 * attributes_.height + 0.1 * meas_attributes.height;
  attributes_.width = 0.9 * attributes_.width + 0.1 * meas_attributes.width;
  attributes_.length = 0.9 * attributes_.length + 0.1 * meas_attributes.length;

  double meas_rot_y = attributes_.rot_y;
  attributes_.rot_y = std::acos(
    rot_cam_to_veh_(0, 0) * std::cos(meas_rot_y) +
    rot_cam_to_veh_(0, 1) * std::sin(meas_rot_y));
}

void Track::set_x(const Eigen::VectorXd& x)
{
  x_ = x;
}

void Track::set_P(const Eigen::MatrixXd& P)
{
  P_ = P;
}

void Track::print() const
{
  std::cout << "Track print ------------------------------" << std::endl;
  std::cout << "id_ = " << std::endl << id_ << std::endl;
  std::cout << "t_ = " << std::endl << t_ << std::endl;
  std::cout << "x_ = " << std::endl << x_ << std::endl;
  std::cout << "P_ = " << std::endl << P_ << std::endl;
  std::cout << "width_ = " << std::endl << attributes_.width << std::endl;
  std::cout << "height_ = " << std::endl << attributes_.height << std::endl;
  std::cout << "length_ = " << std::endl << attributes_.length << std::endl;
  std::cout << "yaw_ = " << std::endl << attributes_.rot_y << std::endl;
}
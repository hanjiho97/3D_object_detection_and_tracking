#include "modules/Tracker.h"


Track::Track(const Measurement& meas, uint id)
{
  id_ = id;
  Eigen::VectorXd z = meas.get_z();
  Eigen::VectorXd pos_cam = Eigen::VectorXd::Ones(4);
  pos_cam.block<3, 1>(0, 0) = z;
  Eigen::MatrixXd cam_to_veh = meas.get_cam_to_veh();
  Eigen::VectorXd pos_veh = cam_to_veh * pos_cam;

  x_ = Eigen::VectorXd::Zero(6);
  x_.block<3, 1>(0, 0) = pos_veh.block<3, 1>(0, 0);

  P_ = Eigen::MatrixXd::Zero(6, 6);
  P_.block<3, 3>(0, 0) = cam_to_veh.block<3, 3>(0, 0) * meas.get_R() *
                         cam_to_veh.block<3, 3>(0, 0).transpose();

  double sigma_p44 = 50.0;
  double sigma_p55 = 50.0;
  double sigma_p66 = 5.0;

  P_(3, 3) = sigma_p44 * sigma_p44;
  P_(4, 4) = sigma_p55 * sigma_p55;
  P_(5, 5) = sigma_p66 * sigma_p66;

  t_ = meas.get_t();
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

void Track::update_attributes(const Measurement& meas) {}

void Track::set_x(const Eigen::VectorXd& x)
{
  x_ = x;
}

void Track::set_P(const Eigen::MatrixXd& P)
{
  P_ = P;
}

void Track::print()
{
  std::cout << "Track print ------------------------------" << std::endl;
  std::cout << "id_ = " << std::endl << id_ << std::endl;
  std::cout << "t_ = " << std::endl << t_ << std::endl;
  std::cout << "x_ = " << std::endl << x_ << std::endl;
  std::cout << "P_ = " << std::endl << P_ << std::endl;
}
#include "modules/Measurement.h"

Measurement::Measurement()
{
  R_ = Eigen::MatrixXd::Zero(3, 3);
  double sigma_lidar_x = 0.1;
  double sigma_lidar_y = 0.1;
  double sigma_lidar_z = 0.1;
  R_(0, 0) = sigma_lidar_x * sigma_lidar_x;
  R_(1, 1) = sigma_lidar_y * sigma_lidar_y;
  R_(2, 2) = sigma_lidar_z * sigma_lidar_z;

  // TODO : inference 결과로부터 멤버변수들 값 할당.
  // 현재는 dummy임
  cam_to_veh_ = Eigen::MatrixXd::Identity(4, 4);
  veh_to_cam_ = cam_to_veh_.inverse();
  fov_.first = - M_PI_2;
  fov_.second = M_PI_2;

  t_ = 1.0;
  z_ = Eigen::VectorXd(3);
  z_ << 1.0, 2.0, 3.0;
  class_num_ = 0;
  width_ = 2.0;
  height_ = 2.0;
  length_ = 2.0;
  yaw_ = 0.0;
}

Measurement::~Measurement() {}

Eigen::VectorXd Measurement::get_z() const
{
  return z_;
}

Eigen::MatrixXd Measurement::get_R() const
{
  return R_;
}

Eigen::VectorXd Measurement::get_hx(const Eigen::VectorXd& x) const
{
  Eigen::VectorXd pos_veh = Eigen::VectorXd::Ones(4);
  pos_veh.block<3, 1>(0, 0) = x.block<3, 1>(0, 0);
  Eigen::VectorXd pos_cam = veh_to_cam_ * pos_veh;
  return pos_cam.block<3, 1>(0, 0);
}

Eigen::MatrixXd Measurement::get_H(const Eigen::VectorXd& x) const
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
  H.block<3, 3>(0, 0) = veh_to_cam_.block<3, 3>(0, 0);
  return H;
}

double Measurement::get_t() const
{
  return t_;
}

bool Measurement::in_fov(const Eigen::VectorXd& x) const
{
  Eigen::VectorXd pos_veh = Eigen::VectorXd::Ones(4);
  pos_veh.block<3, 1>(0, 0) = x.block<3, 1>(0, 0);
  Eigen::VectorXd pos_cam = veh_to_cam_ * pos_veh;

  double angle = std::atan2(pos_cam(1), pos_cam(0));

  if (angle >= fov_.first && angle <= fov_.second)
  {
    return true;
  }
  else
  {
    return false;
  }
}

Eigen::MatrixXd Measurement::get_cam_to_veh() const
{
  return cam_to_veh_;
}

void Measurement::print()
{
  std::cout << "Measurement print ------------------------------" << std::endl;
  std::cout << "z_ = " << std::endl << z_ << std::endl;
  std::cout << "cam_to_veh_ = " << std::endl << cam_to_veh_ << std::endl;
  std::cout << "veh_to_cam_ = " << std::endl << veh_to_cam_ << std::endl;
  std::cout << "R_ = " << std::endl << R_ << std::endl;
  std::cout << "t_ = " << std::endl << t_ << std::endl;
  std::cout << "class_num_ = " << std::endl << class_num_ << std::endl;
  std::cout << "width_ = " << std::endl << width_ << std::endl;
  std::cout << "height_ = " << std::endl << height_ << std::endl;
  std::cout << "length_ = " << std::endl << length_ << std::endl;
  std::cout << "yaw_ = " << std::endl << yaw_ << std::endl;
}
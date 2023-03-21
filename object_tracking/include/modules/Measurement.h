#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <cmath>
#include <iostream>

#include <eigen3/Eigen/Dense>

class Measurement
{
public:
  Measurement();
  virtual ~Measurement();

  Eigen::VectorXd get_z() const;
  Eigen::MatrixXd get_R() const;
  Eigen::VectorXd get_hx(const Eigen::VectorXd& x) const;
  Eigen::MatrixXd get_H(const Eigen::VectorXd& x) const;
  Eigen::MatrixXd get_cam_to_veh() const;
  double get_t() const;

  bool in_fov(const Eigen::VectorXd& x) const;

  void print();

private:
  Eigen::VectorXd z_;  
  Eigen::MatrixXd cam_to_veh_;
  Eigen::MatrixXd veh_to_cam_;
  Eigen::MatrixXd R_;

  std::pair<double, double> fov_;

  double t_;

  uint class_num_;
  double width_;
  double length_;
  double height_;
  double yaw_;
};

#endif  // MEASUREMENT_H_
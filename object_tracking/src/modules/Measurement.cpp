#include "modules/Measurement.h"

Measurement::Measurement() {}

Measurement::~Measurement() {}

Eigen::VectorXd Measurement::get_z() const
{
  return z_;
}

Eigen::MatrixXd Measurement::get_R() const
{
  return R_;
}

Eigen::VectorXd Measurement::get_hx(Eigen::VectorXd x) const {}

Eigen::MatrixXd Measurement::get_H(Eigen::VectorXd x) const {}
#include "object_tracking/Association.h"

Association::Association() {}

Association::~Association() {}

double
Association::MHD(const Track& track, const Measurement& meas, const EKF& ekf)
{
  Eigen::VectorXd y = ekf.get_y(track, meas);
  Eigen::MatrixXd S = ekf.get_S(track, meas);
  Eigen::VectorXd x = track.get_x();
  Eigen::MatrixXd H = meas.get_H(x);

  return (y.transpose() * S.inverse() * y)(0, 0);
}
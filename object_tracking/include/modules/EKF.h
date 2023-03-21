#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

#include "modules/Measurement.h"
#include "modules/Tracker.h"

class EKF
{
public:
  EKF();
  virtual ~EKF();

  void init(const Eigen::MatrixXd& R);
  void predict(const Measurement& meas, Track& track);
  void update(const Measurement& meas, Track& track);

  void set_Q(double delta_t);
  void set_F(double delta_t);

private:
  double delta_t_;

  Eigen::MatrixXd F_;  // state transition matrix
  Eigen::MatrixXd Q_;  // process noise covariance matrix
  Eigen::MatrixXd I_;  // idendity matrix
};
#endif  // EXTENDED_KALMAN_FILTER_H_
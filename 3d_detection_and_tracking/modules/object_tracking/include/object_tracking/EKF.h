#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_

#include <iostream>

#include "Eigen/Dense"

#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"

class Track;

class EKF
{
public:
  EKF();
  virtual ~EKF();
  void predict(uint frame_count, Track& track);
  void update(Track& track, const Measurement& meas);

  void set_Q(double delta_t);
  void set_F(double delta_t);

  Eigen::VectorXd get_y(const Track& track, const Measurement& meas) const;
  Eigen::MatrixXd get_S(const Track& track, const Measurement& meas) const;
  void print() const;

private:
  double delta_t_;

  Eigen::MatrixXd F_;  // state transition matrix
  Eigen::MatrixXd Q_;  // process noise covariance matrix
  Eigen::MatrixXd I_;  // idendity matrix
};

#endif  // EXTENDED_KALMAN_FILTER_H_
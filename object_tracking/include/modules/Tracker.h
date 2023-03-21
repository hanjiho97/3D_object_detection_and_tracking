#ifndef TRACKER_H_
#define TRACKER_H_

#include <eigen3/Eigen/Dense>

#include "modules/Measurement.h"
#include "modules/EKF.h"

class Track
{
public:
  Track(const Measurement& meas, uint id);
  virtual ~Track();

  void update_attributes(const Measurement& meas);

  Eigen::VectorXd get_x();
  Eigen::MatrixXd get_P();
  double get_t();

  void set_x(const Eigen::VectorXd& x);
  void set_P(const Eigen::MatrixXd& P);

  void print();

private:
  uint id_;
  double t_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
};

class TrackManager
{
public:
private:
};

#endif  // TRACKER_H_
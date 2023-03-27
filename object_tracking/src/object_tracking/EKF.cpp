#include "object_tracking/EKF.h"

EKF::EKF()
{
  F_ = Eigen::MatrixXd::Identity(6, 6);
  Q_ = Eigen::MatrixXd::Zero(6, 6);
  I_ = Eigen::MatrixXd::Identity(6, 6);
  delta_t_ = 1.0;
  set_Q(delta_t_);
  set_F(delta_t_);
}

EKF::~EKF() {}


void EKF::predict(uint frame_count, Track& track)
{
  double delta_t = 0.1 * static_cast<double>(frame_count) - track.get_t();
  if (std::abs(delta_t_ - delta_t) > 0.001)
  {
    delta_t_ = delta_t;
    set_Q(delta_t);
    set_F(delta_t);
  }

  Eigen::VectorXd x = track.get_x();
  Eigen::MatrixXd P = track.get_P();

  x = F_ * x;
  P = F_ * P * F_.transpose() + Q_;

  track.set_x(x);
  track.set_P(P);
}

void EKF::update(const Measurement& meas, Track& track)
{
  Eigen::VectorXd x = track.get_x();
  Eigen::MatrixXd P = track.get_P();
  Eigen::VectorXd hx = meas.get_hx(x);
  Eigen::MatrixXd H = meas.get_H(x);
  Eigen::VectorXd z = meas.get_z();
  Eigen::MatrixXd R = meas.get_R();

  Eigen::VectorXd y = z - hx;
  Eigen::MatrixXd Ht = H.transpose();
  Eigen::MatrixXd S = H * P * Ht + R;
  Eigen::MatrixXd K = P * Ht * S.inverse();

  x = x + (K * y);
  P = (I_ - K * H) * P;

  track.set_x(x);
  track.set_P(P);
  track.update_attributes(meas);
}


void EKF::set_F(double delta_t)
{
  F_(0, 3) = delta_t;
  F_(1, 4) = delta_t;
  F_(2, 5) = delta_t;
}

void EKF::set_Q(double delta_t)
{
  double q = 3.0;  // process noise parameter
  double q1 = delta_t * q;
  double q2 = 0.5 * delta_t * delta_t * q;
  double q3 = 1.0 / 3.0 * delta_t * delta_t * delta_t * q;

  Q_(0, 0) = q3;
  Q_(0, 3) = q2;
  Q_(1, 1) = q3;
  Q_(1, 4) = q2;
  Q_(2, 2) = q3;
  Q_(2, 5) = q2;
  Q_(3, 0) = q2;
  Q_(3, 3) = q1;
  Q_(4, 1) = q2;
  Q_(4, 4) = q1;
  Q_(5, 2) = q2;
  Q_(5, 5) = q1;
}

void EKF::print() const
{
  std::cout << "EKF print ------------------------------" << std::endl;
  std::cout << "F_ = " << std::endl << F_ << std::endl;
  std::cout << "Q_ = " << std::endl << Q_ << std::endl;
  std::cout << "I_ = " << std::endl << I_ << std::endl;
}
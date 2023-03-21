#include "modules/EKF.h"

EKF::EKF()
{
}

EKF::~EKF()
{
}

void EKF::init(const Eigen::MatrixXd& R)
{
    R_ = R;
    F_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = Eigen::MatrixXd::Zero(6, 6);
}


void EKF::predict(Track& track)
{
    if(delta_t != track.get_dt())
    {
        delta_t = track.get_dt();
        setQ(delta_t);
        setF(delta_t);
    }

    track.x_ = F_ * track.x_;
    track.P_ = F_ * track.P_ * F_.transpose() + Q_;
}

void EKF::update(const Eigen::VectorXd &z, Track& track)
{
    Eigen::VectorXd hx = track.get_hx();
    Eigen::MatrixXd H = track.get_H();
    Eigen::VectorXd y = z - hx;
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * track.P_ * Ht + R_;
    Eigen::MatrixXd K = track.P_ * Ht * S.inverse();

    track.x_ = track.x_ + (K * y);
    track.P_ = (I_ - K * H) * track.P_;
}


void EKF::setF(double delta_t)
{
    F_(0, 4) = delta_t;
    F_(1, 5) = delta_t;
    F_(2, 6) = delta_t;
}

void EKF::setQ(double delta_t)
{
    double q = 3.0; // process noise parameter
    double q1 = delta_t * q;
    double q2 = 0.5 * delta_t * delta_t * q;
    double q3 = 1 / 3 * delta_t * delta_t * delta_t * q;

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
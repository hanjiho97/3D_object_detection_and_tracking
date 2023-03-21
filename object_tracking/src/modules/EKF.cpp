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


void EKF::predict(double delta_t, Tracker& track)
{
    setQ(delta_t);
    setF(delta_t);
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::update(const Eigen::VectorXd &z)
{
    Eigen::VectorXd hx;
    Eigen::MatrixXd H;
    // TODO : hx, H 받아오기
    Eigen::VectorXd y = z - hx;
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P_ * Ht + R_;
    Eigen::MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + (K * y);
    P_ = (I_ - K * H) * P_;
}


void EKF::setF(double delta_t)
{

}

void EKF::setQ(double delta_t)
{

}
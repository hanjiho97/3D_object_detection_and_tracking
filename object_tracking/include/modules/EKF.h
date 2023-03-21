#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

class EKF
{
public:
    EKF();
    virtual ~EKF();

    void init(const Eigen::MatrixXd& R);
    void predict(double delta_t);
    void update(const Eigen::VectorXd &z);

    void setQ(double delta_t);
    void setF(double delta_t);
private:
    double updated_time;

    Eigen::MatrixXd F_; // state transition matrix
    Eigen::MatrixXd Q_; // process noise covariance matrix
    Eigen::MatrixXd R_; // measurement noise covariance matrix
    Eigen::MatrixXd I_; // idendity matrix
    

};
#endif // EXTENDED_KALMAN_FILTER_H_
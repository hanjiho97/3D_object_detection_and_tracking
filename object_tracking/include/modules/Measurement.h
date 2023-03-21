#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <eigen3/Eigen/Dense>

class Measurement
{
public:
    Measurement();
    virtual ~Measurement();

    Eigen::VectorXd get_z() const;
    Eigen::MatrixXd get_R() const;
    Eigen::VectorXd get_hx(Eigen::VectorXd x) const;
    Eigen::MatrixXd get_H(Eigen::VectorXd x) const;
    double get_t() const;

private:
    Eigen::VectorXd z_; // 3D bbox location (x, y, z)
    Eigen::MatrixXd R_;

    char class_num;
    double width;
    double length;
    double height;
    double yaw;
};

#endif // MEASUREMENT_H_
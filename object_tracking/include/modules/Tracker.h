#ifndef TRACKER_H_
#define TRACKER_H_

#include <eigen3/Eigen/Dense>

#include "modules/EKF.h"
#include "modules/Measurement.h"

class Track
{
public:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;

    Eigen::VectorXd get_hx();
    Eigen::MatrixXd get_H();
    double get_dt();

private:

};

class TrackManager
{
public:

private:

};

#endif // TRACKER_H_
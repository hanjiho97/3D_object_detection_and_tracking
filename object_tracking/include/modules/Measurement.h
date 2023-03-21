#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <eigen3/Eigen/Dense>

class Measurement
{
public:
    Measurement();
    virtual ~Measurement();
    Eigen::VectorXd get_z() const;
private:
    Eigen::VectorXd z_;
};

#endif // MEASUREMENT_H_
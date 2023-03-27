#ifndef ASSOCIATION_H_
#define ASSOCIATION_H_

#include "object_tracking/type.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Tracker.h"
#include "object_tracking/Measurement.h"

class Association
{
public:
    Association();
    virtual ~Association();
    
private:
    double MHD(const Track& track, const Measurement& meas, const EKF& ekf);
};

#endif // ASSOCIATION_H_
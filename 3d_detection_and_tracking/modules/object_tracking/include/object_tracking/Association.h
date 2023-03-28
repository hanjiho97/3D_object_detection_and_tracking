#ifndef ASSOCIATION_H_
#define ASSOCIATION_H_

#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"

class Track;
class EKF;

class Association
{
public:
  Association();
  virtual ~Association();

private:
  double MHD(const Track& track, const Measurement& meas, const EKF& ekf);
};

#endif  // ASSOCIATION_H_
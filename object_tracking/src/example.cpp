#include <eigen3/Eigen/Dense>

#include "modules/EKF.h"
#include "modules/Measurement.h"
#include "modules/Tracker.h"


int main()
{

  EKF ekf = EKF();
  ekf.print();


  Measurement meas = Measurement();
  meas.print();

  Track track = Track(meas, 11);
  track.print();

  meas.z_(0) = 2.0;
  meas.z_(1) = 3.0;
  meas.z_(2) = 4.0;
  meas.t_ = 2.0;

  ekf.predict(meas, track);
  ekf.update(meas, track);
  track.print();


  meas.z_(0) = 3.0;
  meas.z_(1) = 4.0;
  meas.z_(2) = 5.0;
  meas.t_ = 3.0;

    ekf.predict(meas, track);
  ekf.update(meas, track);

  track.print();

  return 0;
}
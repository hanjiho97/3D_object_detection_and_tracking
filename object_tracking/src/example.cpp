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


  return 0;
}
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "modules/EKF.h"
#include "modules/Measurement.h"
#include "modules/Tracker.h"


int main()
{
  std::string kitti_root_path = "../kitti_data/";
  std::string label_path = kitti_root_path + "data/000122.txt";
  std::string calib_path = kitti_root_path + "calib/000122.txt";

  EKF ekf = EKF();
  ekf.print();

  std::vector<Kitti_Object> objs;
  load_kitti_label(label_path, objs);

  Kitti_Calib calib = load_kitti_calib(calib_path);

  std::vector<Measurement> meas_list;
  meas_list.reserve(objs.size());
  int frame_cnt = 0; // 추후에 for loop을 통해 0부터 375 frame까지 반복.
  for(int i = 0; i < objs.size(); ++i)
  {
    Measurement meas(frame_cnt, objs[i], calib);
    meas_list.push_back(meas);
  }
  

  return 0;
}
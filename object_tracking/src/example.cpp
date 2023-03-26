#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"
#include "object_tracking/Association.h"

int main()
{
  std::string kitti_root_path = "../kitti_data/";
  std::string label_path = kitti_root_path + "data/000122.txt";
  std::string calib_path = kitti_root_path + "calib/000122.txt";

  EKF ekf = EKF();
  //ekf.print();

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
  
  std::vector<Track> track_list;
  track_list.reserve(objs.size());
  for(int i = 0; i < objs.size(); ++i)
  {
    Track track(meas_list[i], i);
    track_list.push_back(track);
  }


  for(const auto& track : track_list)
  {
    track.print();
    const Eigen::Ref<const Eigen::VectorXd> x = track.get_x();
    const Attributes& attr = track.get_attributes();
    std::cout << "x : " << x << std:: endl;
    std::cout << "attr" << attr.height << attr.length << attr.width << attr.rot_y << std::endl;
  }

  TrackManager tm = TrackManager();
  tm.add_new_track(meas_list[0]);
  const auto& track_list2 = tm.get_track_list();
  for(const auto& track : track_list2)
  {
    const Attributes& attr = track.get_attributes();
    uint id = track.get_id();
    std::cout << attr.height << " " << id << std::endl;
  }


  return 0;
}
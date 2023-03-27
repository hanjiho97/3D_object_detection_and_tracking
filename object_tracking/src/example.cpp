#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"
#include "object_tracking/Association.h"
#include "object_tracking/Dataloader.h"

int main()
{
  // std::string kitti_root_path = "../kitti_data/";
  // std::string label_path = kitti_root_path + "data/000122.txt";
  // std::string calib_path = kitti_root_path + "calib/000122.txt";

  EKF ekf = EKF();
  //ekf.print();

  uint frame_count = 71;

  Dataloader dataloader;
  kitti::Data test_data;
  test_data = dataloader.get_kitti_data(frame_count);

  std::vector<Measurement> meas_list;
  meas_list.reserve(test_data.labels.size());
  for(int i = 0; i < test_data.labels.size(); ++i)
  {
    Measurement meas(frame_count, i, test_data);
    meas_list.push_back(meas);
  }
  
  std::vector<Track> track_list;
  track_list.reserve(test_data.labels.size());
  for(int i = 0; i < test_data.labels.size(); ++i)
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
    std::cout << "attr" << attr.height << " " << attr.length << " " << attr.width  << " "<< attr.rot_y << std::endl;
  }

  TrackManager tm = TrackManager();
  tm.add_new_track(meas_list[0]);
  const auto& track_list2 = tm.get_track_list();
  for(const auto& pair : track_list2)
  {
    const Attributes& attr = pair.second.get_attributes();
    uint id = pair.second.get_id();
    std::cout << attr.height << " " << id << std::endl;
  }

  return 0;
}
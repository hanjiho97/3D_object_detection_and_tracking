#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"
#include "object_tracking/Association.h"
#include "object_tracking/Dataloader.h"

int main()
{


  Dataloader data_loader = Dataloader();
  kitti::Data kitti_data;
  TrackManager track_manager = TrackManager();
  Association association = Association();
  EKF ekf = EKF();
  std::vector<Measurement> meas_list;

  for(uint frame_count = 0; frame_count <= 375; ++frame_count)
  {
    kitti_data = data_loader.get_kitti_data(frame_count);
    meas_list.clear();
    meas_list.reserve(kitti_data.labels.size());
    for(uint label_num = 0; label_num < kitti_data.labels.size(); ++label_num)
    {
      Measurement meas = Measurement(frame_count, label_num, kitti_data);
      meas_list.push_back(meas);
    }

    track_manager.predict_tracks(frame_count, ekf);
    if(meas_list.empty())
    {
      continue;
    }
    association.associate_and_update(track_manager, meas_list, ekf);

    std::map<uint, Track> track_list = track_manager.get_track_list();
    for(auto& track_pair : track_list)
    {
      track_pair.second.get_attributes();
    }
  }



  return 0;
}
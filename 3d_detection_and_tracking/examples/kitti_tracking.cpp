#include <iostream>
#include <vector>

#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/Association.h"
#include "object_tracking/Dataloader.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"
#include "result_viewer/Viewer.h"
#include "result_viewer/Projection.h"

int main()
{
  kitti::Data kitti_data;
  Dataloader data_loader = Dataloader();
  TrackManager track_manager = TrackManager();
  Association association = Association();
  EKF ekf = EKF();
  Viewer viewer = Viewer();
  std::vector<Measurement> meas_list;
  bool show_bbox_3D=true;
  bool showing_head=true;
  bool showing_id=true;
  bool showing_topview=true;

  for(uint frame_count = 0; frame_count <= 470; ++frame_count)
  {
    kitti_data = data_loader.get_kitti_data(frame_count);
    viewer.read_P2_matrix(kitti_data.calibration.P2);
    viewer.add_image(kitti_data.image);
    meas_list.clear();
    meas_list.reserve(kitti_data.labels.size());
    for(uint label_num = 0; label_num < kitti_data.labels.size(); ++label_num)
    {
      Measurement meas = Measurement(frame_count, label_num, kitti_data);
      meas_list.push_back(meas);
    }

    track_manager.predict_tracks(frame_count, ekf);
    association.associate_and_update(track_manager, meas_list, ekf);
    std::map<uint, Track> track_list = track_manager.get_track_list();
    for(auto& track_pair : track_list)
    {
      if (track_pair.second.get_state() == 2)
      {
        viewer.add_3d_bbox(track_pair.first, track_pair.second.get_attributes());
        viewer.draw(
        show_bbox_3D, 
        showing_head, 
        showing_id,
        showing_topview);
      }
    }
    viewer.show_result();
  }
  return 0;
}

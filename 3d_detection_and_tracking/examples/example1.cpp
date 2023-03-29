#include <iostream>
#include <vector>

#include "Eigen/Dense"

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
  Dataloader data_loader = Dataloader();
  kitti::Data kitti_data;
  TrackManager track_manager = TrackManager();
  Association association = Association();
  EKF ekf = EKF();
  std::vector<Measurement> meas_list;
  Viewer viewer;
  Projection projection;

  for(uint frame_count = 0; frame_count <= 1058; ++frame_count)
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
    association.associate_and_update(track_manager, meas_list, ekf);

    Attributes attributes;
    std::vector<uint> id_list;
    std::vector<cv::Point> points;
    std::vector<std::vector<cv::Point>> points_list;
    bool show_bbox_3D = false;
    bool showing_head = false;
    bool showing_id = false;
    std::map<uint, Track> track_list = track_manager.get_track_list();
    for(auto& track_pair : track_list)
    {
      if (track_pair.second.get_state() == 2)
      {
        attributes = track_pair.second.get_attributes();
        projection.read_data(attributes, kitti_data.calibration.P2);
        points = projection.get_2D_corners();
        points_list.push_back(points);
        id_list.push_back(track_pair.first);
      }
    }
    if (points_list.size() > 0)
    {
      show_bbox_3D = true;
      showing_head = true;
      showing_id = true;
    }
    viewer.read_data(kitti_data.image, points_list, id_list);
    viewer.show_result(show_bbox_3D, showing_head, showing_id);
  }



  return 0;
}
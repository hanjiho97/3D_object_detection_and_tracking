#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/Association.h"
#include "object_tracking/Dataloader.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"
#include "result_viewer/Viewer.h"

int main()
{
  
  Dataloader data_loader = Dataloader();
  TrackManager track_manager = TrackManager();
  Association association = Association();
  EKF ekf = EKF();
  Viewer viewer = Viewer();
  std::vector<Measurement> measurement_list;

  //Viewer variables
  bool show_bbox_3D = true;
  bool showing_head = true;
  bool showing_id = true;
  bool showing_topview_box = true;
  bool showing_topview_id = true;
  bool showing_topview_position = true;
  bool showing_topview_car = true;

  const uint16_t TOTAL_FRAME_COUNT = 447;

  for(uint frame_count = 0; frame_count < TOTAL_FRAME_COUNT; ++frame_count)
  {
    //Load kitti data using dataloader
    kitti::Data kitti_data;
    kitti_data = data_loader.get_kitti_data(frame_count);

    //Initialize measurement list
    Measurement::kitti_to_measurement_list(frame_count, kitti_data, measurement_list);

    //Track objects
    track_manager.predict_tracks(frame_count, ekf);
    association.associate_and_update(track_manager, measurement_list, ekf);
    std::map<uint, Attributes> attributes_list = track_manager.get_attributes();

    //View the result
    viewer.read_P2_matrix(kitti_data.calibration.P2);
    viewer.add_image(kitti_data.image);
    viewer.add_attributes_list(attributes_list);
    viewer.show_result(
        show_bbox_3D,
        showing_head,
        showing_id,
        showing_topview_box,
        showing_topview_id,
        showing_topview_position,
        showing_topview_car);
  }
  return 0;
}

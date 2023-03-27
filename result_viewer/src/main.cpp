#include "result_viewer/Dataloader.h"
#include "result_viewer/Projection.h"
#include "result_viewer/Type.h"
#include "result_viewer/Viewer.h"

int main(void)
{
  Dataloader dataloader;
  Viewer viewer;
  Projection projection;

  kitti::Data test_data;
  test_data = dataloader.get_kitti_data(71);
  Attributes attributes;
  attributes.loc_x = test_data.labels[0].loc_x;
  attributes.loc_y = test_data.labels[0].loc_y;
  attributes.loc_z = test_data.labels[0].loc_z;
  attributes.height = test_data.labels[0].height;
  attributes.width = test_data.labels[0].width;
  attributes.length = test_data.labels[0].length;
  attributes.rot_y = test_data.labels[0].rot_y;

  std::vector<cv::Point> points;
  projection.read_data(attributes, test_data.calibration.P2);
  points = projection.get_2D_corners();

  bool show_bbox_3D = true;
  bool showing_head = true;
  viewer.read_data(test_data.image, points);
  viewer.show_result(show_bbox_3D, showing_head);

  return 0;
}
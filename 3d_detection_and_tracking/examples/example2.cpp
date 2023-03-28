#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/Dataloader.h"
#include "result_viewer/Viewer.h"
#include "result_viewer/Projection.h"

int main(void)
{
  Dataloader dataloader;
  Viewer viewer;
  Projection projection;

  kitti::Data test_data;
  for(int index=0; index < 376; ++index)
  {
    test_data = dataloader.get_kitti_data(index);
    Attributes attributes;
    std::vector<cv::Point> points;
    std::vector<std::vector<cv::Point>> points_list;
    bool show_bbox_3D = false;
    bool showing_head = false;
    if (test_data.labels.size() > 0)
    {
      for (int i=0; i<test_data.labels.size(); ++i)
      {
        attributes.loc_x = test_data.labels[i].loc_x;
        attributes.loc_y = test_data.labels[i].loc_y;
        attributes.loc_z = test_data.labels[i].loc_z;
        attributes.height = test_data.labels[i].height;
        attributes.width = test_data.labels[i].width;
        attributes.length = test_data.labels[i].length;
        attributes.rot_y = test_data.labels[i].rot_y;
        projection.read_data(attributes, test_data.calibration.P2);
        points = projection.get_2D_corners();
        points_list.push_back(points);
      }
      bool show_bbox_3D = true;
      bool showing_head = true;
      viewer.read_data(test_data.image, points_list);
      viewer.show_result(show_bbox_3D, showing_head);
    }
    else
    {
      viewer.read_data(test_data.image, points_list);
      viewer.show_result(show_bbox_3D, showing_head);
    }
  }

  return 0;
}
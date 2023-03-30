#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/Dataloader.h"
#include "result_viewer/Viewer.h"

int main(void)
{
  Dataloader dataloader;
  Viewer viewer;
  bool show_bbox_3D=true;
  bool showing_head=true;
  bool showing_id=true;
  bool showing_topview=true;
  for (int frame_count=0; frame_count < 1058; ++frame_count)
  {
    kitti::Data test_data;
    test_data = dataloader.get_kitti_data(frame_count);
    viewer.read_P2_matrix(test_data.calibration.P2);
    viewer.add_image(test_data.image);
    Attributes attributes;
    if (test_data.labels.size() > 0)
    {
      for (int index=0; index < test_data.labels.size(); ++index)
      {
        attributes.loc_x = test_data.labels[index].loc_x;
        attributes.loc_y = test_data.labels[index].loc_y;
        attributes.loc_z = test_data.labels[index].loc_z;
        attributes.height = test_data.labels[index].height;
        attributes.width = test_data.labels[index].width;
        attributes.length = test_data.labels[index].length;
        attributes.rot_y = test_data.labels[index].rot_y;
        uint16_t id = index;
        viewer.add_3d_bbox(id, attributes);
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

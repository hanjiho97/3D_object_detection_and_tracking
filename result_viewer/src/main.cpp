#include "result_viewer/Dataloader.h"
#include "result_viewer/Type.h"
#include "result_viewer/Viewer.h"

int main(void)
{
  Dataloader dataloader;
  Viewer viewer;
  bool show_bbox_3D = true;
  bool showing_head = true;
  bool showing_id = true;
  bool showing_topview_box = true;
  bool showing_topview_id = true;
  bool showing_topview_position = true;
  bool showing_topview_car = true;
  const uint16_t TOTAL_FRAME_COUNT = 447;
  for (int frame_count = 0; frame_count < TOTAL_FRAME_COUNT; ++frame_count)
  {
    kitti::Data test_data;
    test_data = dataloader.get_kitti_data(frame_count);
    Attributes attributes;
    std::map<uint, Attributes> attributes_list;
    if (test_data.labels.size() > 0)
    {
      for (int index = 0; index < test_data.labels.size(); ++index)
      {
        attributes.loc_x = test_data.labels[index].loc_x;
        attributes.loc_y = test_data.labels[index].loc_y;
        attributes.loc_z = test_data.labels[index].loc_z;
        attributes.height = test_data.labels[index].height;
        attributes.width = test_data.labels[index].width;
        attributes.length = test_data.labels[index].length;
        attributes.rot_y = test_data.labels[index].rot_y;
        attributes_list.insert({index, attributes});
      }
      viewer.read_P2_matrix(test_data.calibration.P2);
      viewer.add_image(test_data.image);
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
  }
  return 0;
}

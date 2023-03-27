#include "result_viewer/Dataloader.h"

int main(void)
{
  Dataloader dataloader;
  kitti::Data test_data;
  test_data = dataloader.get_kitti_data(71);
  std::cout << test_data.labels[0].loc_x << std::endl;
  std::cout << test_data.calibration.P0 << std::endl;
  // cv::imshow("test", test_data.image);
  // cv::waitKey(1000);
  return 0;
}
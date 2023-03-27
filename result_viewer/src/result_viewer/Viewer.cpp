#include "result_viewer/Viewer.h"

Viewer::Viewer() {}

Viewer::~Viewer() {}

void Viewer::show_result(cv::Mat& image, bool bbox_3D_flag)
{
  if (bbox_3D_flag == true)
  {
    std::cout << "test" << std::endl;
  }
  cv::imshow("result_image", image);
  cv::waitKey();
}

bool Viewer::draw_3d_bbox(cv::Mat& image, const std::vector<cv::Point>& bbox_points)
{
  return 0;
}
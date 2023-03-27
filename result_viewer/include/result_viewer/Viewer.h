#ifndef VIEWER_H
#define VIEWER_H

#include <vector>
#include "opencv2/opencv.hpp"

class Viewer
{
public:
  Viewer();
  virtual ~Viewer();
  bool show_image_with_3d_bbox(const cv::Mat& frame);
  bool draw_3d_bbox(cv::Mat& frame, const std::vector<cv::Point>& 3d_bbox_points);
private:
  cv::Mat _frame;
};

#endif
#ifndef VIEWER_H
#define VIEWER_H

#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"

class Viewer
{
public:
  Viewer();
  virtual ~Viewer();
  void show_result(cv::Mat& image, bool bbox_3D_flag);
  bool draw_3d_bbox(cv::Mat& image, const std::vector<cv::Point>& bbox_points);
};

#endif
#ifndef VIEWER_H
#define VIEWER_H

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"

#include "result_viewer/Projection.h"

class Viewer
{
public:
  Viewer();
  virtual ~Viewer();
  void show_result(bool show_bbox_3D, bool showing_head);
  void draw_3d_bbox(cv::Mat& image, const std::vector<cv::Point>& bbox_points);
  void draw_3d_bbox_head(
    cv::Mat& image,
    const std::vector<cv::Point>& bbox_points);

  void read_data(
    const cv::Mat& image,
    const std::vector<cv::Point>& bbox_points);

private:
  std::vector<cv::Point> bbox_points_;
  cv::Mat image_;
};

#endif
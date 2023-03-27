#include "result_viewer/Viewer.h"

Viewer::Viewer() {}

Viewer::~Viewer() {}

void Viewer::show_result(bool show_bbox_3D, bool showing_head)
{
  if (show_bbox_3D == true)
  {
    draw_3d_bbox(image_, bbox_points_);
  }
  if (showing_head == true)
  {
    draw_3d_bbox_head(image_, bbox_points_);
  }
  cv::imshow("result_image", image_);
  cv::waitKey();
}

void Viewer::draw_3d_bbox(
  cv::Mat& image,
  const std::vector<cv::Point>& bbox_points)
{
  for (uint8_t index = 0; index < bbox_points.size() - 4; index += 2)
  {
    cv::line(
      image,
      bbox_points[index],
      bbox_points[index + 1],
      cv::Scalar(255, 0, 255),
      2);
  }
  return;
}

void Viewer::draw_3d_bbox_head(
  cv::Mat& image,
  const std::vector<cv::Point>& bbox_points)
{
  cv::line(image, bbox_points[24], bbox_points[25], cv::Scalar(255, 0, 255), 2);
  cv::line(image, bbox_points[26], bbox_points[27], cv::Scalar(255, 0, 255), 2);
  return;
}

void Viewer::read_data(
  const cv::Mat& image,
  const std::vector<cv::Point>& bbox_points)
{
  image_ = image;
  bbox_points_ = bbox_points;
  return;
}
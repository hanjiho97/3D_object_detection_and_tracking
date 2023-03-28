#include "result_viewer/Viewer.h"

Viewer::Viewer() {}

Viewer::~Viewer() {}

void Viewer::show_result(bool show_bbox_3D, bool showing_head)
{
  if (show_bbox_3D == true)
  {
    draw_3d_bbox(image_, bbox_points_list_);
  }
  if (showing_head == true)
  {
    draw_3d_bbox_head(image_, bbox_points_list_);
  }
  cv::imshow("result_image", image_);
  cv::waitKey(75);
}

void Viewer::draw_3d_bbox(
  cv::Mat& image,
  const std::vector<std::vector<cv::Point>>& bbox_points_list)
{
  for (auto bbox_points : bbox_points_list)
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
  }
  return;
}

void Viewer::draw_3d_bbox_head(
  cv::Mat& image,
  const std::vector<std::vector<cv::Point>>& bbox_points_list)
{
  for (auto bbox_points : bbox_points_list)
  {
    cv::line(image, bbox_points[24], bbox_points[25], cv::Scalar(255, 0, 255), 2);
    cv::line(image, bbox_points[26], bbox_points[27], cv::Scalar(255, 0, 255), 2);
  }
  return;
}

void Viewer::read_data(
  const cv::Mat& image,
  const std::vector<std::vector<cv::Point>>& bbox_points_list)
{
  image_ = image;
  bbox_points_list_ = bbox_points_list;
  return;
}
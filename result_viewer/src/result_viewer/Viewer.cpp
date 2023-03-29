#include "result_viewer/Viewer.h"

Viewer::Viewer() {}

Viewer::~Viewer() {}

void Viewer::generate_color_map()
{
  std::mt19937 e2(1000);
  cv::Scalar color;
  uint8_t R, G, B;
  for (uint16_t index=0; index < 100; ++index)
  {
    R = e2() % 256;
    G = e2() % 256;
    B = e2() % 256;
    color = cv::Scalar(R, G, B);
    color_map_.push_back(color);
  }
}

void Viewer::draw_3d_bbox(
  const std::vector<cv::Point>& bbox_points,
  bool showing_head
  )
{
  uint16_t size = bbox_points.size();
  if (showing_head == false)
  {
    size -= 4;
  }
  for (uint8_t index = 0; index < size; index += 2)
  {
    cv::line(
      image_,
      bbox_points[index],
      bbox_points[index + 1],
      color_map_[id_%100],
      2);
  }
}

void Viewer::draw_id(cv::Point& centor_point)
{
  cv::putText(
    image_, 
    std::to_string(id_), 
    centor_point, 
    cv::FONT_HERSHEY_DUPLEX, 
    1, 
    color_map_[id_%100], 
    1, 
    4);
}

void Viewer::draw(
  bool show_bbox_3D=true,
  bool showing_head=true, 
  bool showing_id=true)
{
  std::vector<cv::Point> bbox_points;
  bbox_points = projection_.get_2D_corners(attributes_, P2_);

  cv::Point centor_point;
  centor_point = (bbox_points[24] + bbox_points[27]) / 2;
  centor_point.y -= 10;
  if (show_bbox_3D == true)
  {
    draw_3d_bbox(bbox_points, showing_head);
  }

  if (showing_id == true)
  {
    draw_id(centor_point);
  }
}

void Viewer::read_P2_matrix(const Eigen::Matrix<double, 3, 4>& P2)
{
  P2_ = P2;
}

void Viewer::add_image(const cv::Mat& image)
{
  image_ = image;
}

void Viewer::add_3d_bbox(
  uint16_t id,
  const Attributes& attributes)
{
  id_ = id;
  attributes_ = attributes;
}

void Viewer::show_result()
{
  cv::imshow("result_image", image_);
  cv::waitKey(75);
}
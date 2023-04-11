#include "result_viewer/Viewer.h"

Viewer::Viewer()
{
  load_resource_images("../modules/result_viewer/resource/");
  generate_color_map();
}

Viewer::~Viewer() {}

void Viewer::generate_color_map()
{
  std::mt19937 e2(1000);
  cv::Scalar color;
  uint8_t R, G, B;
  for (uint8_t index = 0; index < COLOR_MAP_SIZE; ++index)
  {
    R = e2() % 256;
    G = e2() % 256;
    B = e2() % 256;
    color = cv::Scalar(R, G, B);
    color_map_.push_back(color);
  }
}

bool Viewer::load_resource_images(const std::string& resource_path)
{
  background_ = cv::imread(resource_path + "background.png");
  if (background_.empty())
  {
    spdlog::error(
      "cannot load image from resource_path : {}{}",
      resource_path,
      "background.png");
    return -1;
  }
  cv::Mat car_image;
  std::string file_name;
  for (uint8_t car_number = 1; car_number < 15; ++car_number)
  {
    file_name = "car" + std::to_string(car_number) + ".png";
    car_image = cv::imread(resource_path + file_name);
    if (background_.empty())
    {
      spdlog::error(
        "cannot load image from resource_path : {}{}",
        resource_path,
        file_name);
      return -1;
    }
    car_list_.push_back(car_image);
  }
  return 0;
}

void Viewer::draw_3d_bbox(
  const std::vector<cv::Point>& bbox_points,
  bool showing_head)
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
      color_map_[id_ % 100],
      LINE_THICKNESS);
  }
}

void Viewer::draw_id(const std::vector<cv::Point>& bbox_points)
{
  cv::Point centor_point;
  centor_point = (bbox_points[24] + bbox_points[27]) / 2;
  centor_point.y -= 10;
  cv::putText(
    image_,
    std::to_string(id_),
    centor_point,
    cv::FONT_HERSHEY_DUPLEX,
    FONT_SCALE_07,
    color_map_[id_ % 100],
    TEXT_THICKNESS,
    cv::LINE_4);
}

void Viewer::draw_topview_box(const std::vector<cv::Point>& topview_bbox_points)
{
  for (uint8_t index = 0; index < topview_bbox_points.size() - 1; ++index)
  {
    cv::line(
      top_view_,
      topview_bbox_points[index],
      topview_bbox_points[index + 1],
      color_map_[id_ % 100],
      LINE_THICKNESS,
      cv::LINE_AA);
  }
}

void Viewer::draw_topview_id(const std::vector<cv::Point>& topview_bbox_points)
{
  cv::Point centor_point =
    (topview_bbox_points[2] + topview_bbox_points[3]) / 2;
  if (topview_bbox_points[1].y > (topview_bbox_points[2].y + 30))
  {
    centor_point.y -= 5;
    centor_point.x -= 15;
  }
  else if ((topview_bbox_points[1].y + 30) < topview_bbox_points[2].y)
  {
    centor_point.y += 15;
    centor_point.x -= 15;
  }
  else
  {
    centor_point.y += 5;
    if (topview_bbox_points[1].x > topview_bbox_points[2].x)
    {
      centor_point.x -= 35;
    }
    else
    {
      centor_point.x += 5;
    }
  }
  cv::putText(
    top_view_,
    "id:" + std::to_string(id_),
    centor_point,
    cv::FONT_HERSHEY_DUPLEX,
    FONT_SCALE_05,
    color_map_[id_ % 100],
    TEXT_THICKNESS,
    cv::LINE_AA);
}

void Viewer::draw_topview_position(
  const std::vector<cv::Point>& topview_bbox_points)
{
  cv::Point centor_point =
    (topview_bbox_points[3] + topview_bbox_points[4]) / 2;
  if (topview_bbox_points[3].y > (topview_bbox_points[4].y + 30))
  {
    centor_point.y -= 10;
    centor_point.x -= 65;
  }
  else if ((topview_bbox_points[3].y + 30) < topview_bbox_points[4].y)
  {
    centor_point.y -= 10;
    centor_point.x += 10;
  }
  else
  {
    centor_point.x -= 25;
    if (topview_bbox_points[1].x > topview_bbox_points[2].x)
    {
      centor_point.y -= 30;
    }
    else
    {
      centor_point.y += 15;
    }
  }
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << attributes_.loc_x;
  cv::putText(
    top_view_,
    "x:" + ss.str() + "m",
    centor_point,
    cv::FONT_HERSHEY_DUPLEX,
    FONT_SCALE_04,
    color_map_[id_ % 100],
    TEXT_THICKNESS,
    cv::LINE_AA);

  ss.str("");
  ss.clear();
  ss << std::fixed << std::setprecision(2) << attributes_.loc_z;
  centor_point.y += 12;
  cv::putText(
    top_view_,
    "y:" + ss.str() + "m",
    centor_point,
    cv::FONT_HERSHEY_DUPLEX,
    FONT_SCALE_04,
    color_map_[id_ % 100],
    TEXT_THICKNESS,
    cv::LINE_AA);

  ss.str("");
  ss.clear();
  ss << std::fixed << std::setprecision(2) << attributes_.loc_y;
  centor_point.y += 12;
  cv::putText(
    top_view_,
    "z:" + ss.str() + "m",
    centor_point,
    cv::FONT_HERSHEY_DUPLEX,
    FONT_SCALE_04,
    color_map_[id_ % 100],
    TEXT_THICKNESS,
    cv::LINE_AA);
}

void Viewer::draw_topview_car(const std::vector<cv::Point>& topview_bbox_points)
{
  cv::Point max_point = {0, 0};
  cv::Point min_point = {BACKGROUND_WIDTH - 1, BACKGROUND_HEIGHT - 1};
  cv::Mat car = car_list_[id_ % 14];
  cv::Mat moved_car;
  cv::Mat roi_moved_car;
  cv::Mat car_mask;
  cv::Mat roi_car_mask;
  cv::Mat roi_topview;

  std::vector<cv::Point2f> src_point;
  std::vector<cv::Point2f> dst_point;

  for (int index = 0; index < 4; ++index)
  {
    dst_point.push_back(
      cv::Point2f(topview_bbox_points[index].x, topview_bbox_points[index].y));
    max_point.x = std::max(max_point.x, topview_bbox_points[index].x);
    max_point.y = std::max(max_point.y, topview_bbox_points[index].y);
    min_point.x = std::min(min_point.x, topview_bbox_points[index].x);
    min_point.y = std::min(min_point.y, topview_bbox_points[index].y);
  }
  max_point.x = std::min(max_point.x, BACKGROUND_WIDTH - 1);
  max_point.y = std::min(max_point.y, BACKGROUND_HEIGHT - 1);
  min_point.x = std::max(min_point.x, 0);
  min_point.y = std::max(min_point.y, 0);

  src_point.push_back(cv::Point2f(car.cols - 1, 0));
  src_point.push_back(cv::Point2f(0, 0));
  src_point.push_back(cv::Point2f(0, car.rows - 1));
  src_point.push_back(cv::Point2f(car.cols - 1, car.rows - 1));
  cv::Mat perspective_matrix =
    cv::getPerspectiveTransform(src_point, dst_point);
  cv::warpPerspective(
    car,
    moved_car,
    perspective_matrix,
    cv::Size(BACKGROUND_HEIGHT, BACKGROUND_WIDTH));
  roi_moved_car = moved_car(cv::Rect(max_point, min_point));

  cv::cvtColor(moved_car, car_mask, cv::COLOR_BGR2GRAY);
  cv::threshold(car_mask, car_mask, 0, 255, cv::THRESH_BINARY_INV);
  cv::cvtColor(car_mask, car_mask, cv::COLOR_GRAY2BGR);
  roi_car_mask = car_mask(cv::Rect(max_point, min_point));

  roi_topview = top_view_(cv::Rect(max_point, min_point));
  cv::bitwise_and(roi_topview, roi_car_mask, roi_topview);
  cv::bitwise_or(roi_topview, roi_moved_car, roi_topview);
}

void Viewer::draw(
  bool show_bbox_3D,
  bool showing_head,
  bool showing_id,
  bool showing_topview_box,
  bool showing_topview_id,
  bool showing_topview_position,
  bool showing_topview_car)
{
  std::vector<cv::Point> bbox_points;
  std::vector<cv::Point> top_view_bbox_points;
  for (auto& attributes_pair : attributes_list_)
  {
    id_ = attributes_pair.first;
    attributes_ = attributes_pair.second;
    bbox_points = projection_.get_2D_corners(attributes_, P2_);
    top_view_bbox_points = projection_.get_topview_conrers(
      BACKGROUND_HALF_WIDTH, BACKGROUND_HEIGHT, BACKGROUND_BOX_SCALE);
    spdlog::debug("bboxpoints size : {}", bbox_points.size());
    spdlog::debug("topview bboxpoints size : {}", top_view_bbox_points.size());
    if (show_bbox_3D == true)
    {
      draw_3d_bbox(bbox_points, showing_head);
    }
    if (showing_id == true)
    {
      draw_id(bbox_points);
    }
    if (showing_topview_box == true)
    {
      draw_topview_box(top_view_bbox_points);
    }
    if (showing_topview_id == true)
    {
      draw_topview_id(top_view_bbox_points);
    }
    if (showing_topview_position == true)
    {
      draw_topview_position(top_view_bbox_points);
    }
    if (showing_topview_car == true)
    {
      draw_topview_car(top_view_bbox_points);
    }
  }
}

void Viewer::read_P2_matrix(const Eigen::Matrix<double, 3, 4>& P2)
{
  P2_ = P2;
}

void Viewer::add_image(const cv::Mat& image)
{
  image_ = image;
  background_.copyTo(top_view_);
}

void Viewer::add_attributes_list(
  const std::map<uint, Attributes>& attributes_list)
{
  attributes_list_ = attributes_list;
}

void Viewer::show_result(
  bool show_bbox_3D,
  bool showing_head,
  bool showing_id,
  bool showing_topview_box,
  bool showing_topview_id,
  bool showing_topview_position,
  bool showing_topview_car)
{
  draw(
    show_bbox_3D,
    showing_head,
    showing_id,
    showing_topview_box,
    showing_topview_id,
    showing_topview_position,
    showing_topview_car);
  if (
    (showing_topview_box == true) || (showing_topview_id == true) ||
    (showing_topview_position == true) || (showing_topview_car == true))
  {
    cv::imshow("top_view", top_view_);
  }
  cv::imshow("result_image", image_);
  cv::waitKey(0);
}

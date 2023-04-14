#ifndef VIEWER_H
#define VIEWER_H

#include <cstdint>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

#include "3d_detection_and_tracking/Type.h"
#include "result_viewer/Projection.h"

class Viewer
{
public:
  Viewer();
  virtual ~Viewer();
  void generate_color_map();
  bool load_resource_images(const std::string& resource_path);

  void draw_3d_bbox(
    const std::vector<cv::Point>& bbox_points,
    bool showing_head);
  void draw_id(const std::vector<cv::Point>& bbox_points);

  void draw_topview_box(const std::vector<cv::Point>& topview_bbox_points);
  void draw_topview_id(const std::vector<cv::Point>& topview_bbox_points);
  void draw_topview_position(const std::vector<cv::Point>& topview_bbox_points);
  void draw_topview_car(const std::vector<cv::Point>& topview_bbox_points);
  void draw(
    bool show_bbox_3D,
    bool showing_head,
    bool showing_id,
    bool showing_topview_box,
    bool showing_topview_id,
    bool showing_topview_position,
    bool showing_topview_car);

  void read_P2_matrix(const Eigen::Matrix<double, 3, 4>& P2);
  void add_image(const cv::Mat& image);
  void add_attributes_list(
    const std::map<uint32_t, Attributes>& attributes_list);
  void show_result(
    bool show_bbox_3D,
    bool showing_head,
    bool showing_id,
    bool showing_topview_box,
    bool showing_topview_id,
    bool showing_topview_position,
    bool showing_topview_car);

private:
  cv::Mat image_;
  cv::Mat top_view_;
  cv::Mat background_;
  Eigen::Matrix<double, 3, 4> P2_;
  std::vector<cv::Scalar> color_map_;
  std::vector<cv::Mat> car_list_;
  uint32_t id_;
  Attributes attributes_;
  std::map<uint32_t, Attributes> attributes_list_;
  Projection projection_;

  const uint16_t BACKGROUND_HEIGHT = 600;
  const uint16_t BACKGROUND_WIDTH = 600;
  const uint16_t BACKGROUND_HALF_WIDTH = 300;
  const uint8_t BACKGROUND_BOX_SCALE = 15;
  const uint8_t COLOR_MAP_SIZE = 100;
  const uint8_t LINE_THICKNESS = 2;
  const uint8_t TEXT_THICKNESS = 1;
  const double FONT_SCALE_07 = 0.7;
  const double FONT_SCALE_05 = 0.5;
  const double FONT_SCALE_04 = 0.4;
};

#endif

#ifndef VIEWER_H
#define VIEWER_H

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <random>
#include "opencv2/opencv.hpp"

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
  void draw_id(const cv::Point& centor_point);

  void draw_topview(const std::vector<cv::Point>& topview_bbox_points);
  void draw_topview_id(const cv::Point& centor_point);
  void draw_topview_position(cv::Point& centor_point);
  void draw_topview_car(const std::vector<cv::Point>& topview_bbox_points);
  void draw(
    bool show_bbox_3D, 
    bool showing_head, 
    bool showing_id,
    bool showing_topview);

  void read_P2_matrix(const Eigen::Matrix<double, 3, 4>& P2);
  void add_image(const cv::Mat& image);
  void add_3d_bbox(
    uint16_t id,
    const Attributes& attributes);
  void show_result();

private:
  uint8_t id_;
  cv::Mat image_;
  cv::Mat background_;
  Eigen::Matrix<double, 3, 4> P2_;
  std::vector<cv::Scalar> color_map_;
  std::vector<cv::Mat> car_list_;
  Attributes attributes_;
  Projection projection_;
};

#endif
#ifndef DATALOADER_H_
#define DATALOADER_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

namespace kitti
{
struct Label
{
  std::string type;
  double truncated;
  double occluded;
  double alpha;
  double left, top, right, bottom;  // 2D bbox
  double height, width, length;
  double loc_x, loc_y, loc_z;  // camera coordinates
  double rot_y;
  double score;
};

struct Calibration
{
  Eigen::Matrix<double, 3, 4> P0, P1, P2, P3;
  Eigen::Matrix<double, 3, 3> R0_rect;
  Eigen::Matrix<double, 3, 4> velo_to_cam;
  Eigen::Matrix<double, 3, 4> imu_to_velo;
};

struct Data
{
  Calibration calibration;
  cv::Mat image;
  std::vector<kitti::Label> labels;
};
}  //namespace Kitti

class Dataloader
{
public:
  Dataloader();
  virtual ~Dataloader();

  bool load_kitti_calibration(
    const std::string& calibration_path, 
    kitti::Calibration& calibration_data);
  bool load_kitti_image(
    const std::string& image_path,
    cv::Mat& image);
  bool load_kitti_label(
    const std::string& label_path,
    std::vector<kitti::Label>& labels);

  kitti::Data get_kitti_data(uint frame_count);

private:
  uint string_length_;
  uint frame_count_;
  std::string kitti_root_path_;
  std::string calibration_path_;
  std::string image_path_;
  std::string label_path_;
};

#endif  // DATALOADER_H_

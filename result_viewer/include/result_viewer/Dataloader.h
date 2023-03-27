#ifndef DATALOADER_H_
#define DATALOADER_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "result_viewer/Type.h"

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

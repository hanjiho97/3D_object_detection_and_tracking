#include "result_viewer/Dataloader.h"

Dataloader::Dataloader()
{
  string_length_ = 6;
  frame_count_ = 0;
  kitti_root_path_ = "../kitti_data";
  calibration_path_ = "/calib/";
  image_path_ = "/image_2/";
  label_path_ = "/label_2/";
}

Dataloader::~Dataloader() {}

bool Dataloader::load_kitti_calibration(
  const std::string& calibration_path,
  kitti::Calibration& calibration_data)
{
  std::ifstream ifs(calibration_path);
  std::string line;
  double value;
  if (!ifs.is_open())
  {
    std::cout << "cannot open calib_path : " << calibration_path << std::endl;
  }

  std::getline(ifs, line);
  std::stringstream ss(line.substr(4));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      ss >> value;
      calibration_data.P0(row, col) = value;
    }
  }

  std::getline(ifs, line);
  ss.clear();
  ss.str(line.substr(4));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      ss >> value;
      calibration_data.P1(row, col) = value;
    }
  }

  std::getline(ifs, line);
  ss.clear();
  ss.str(line.substr(4));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      ss >> value;
      calibration_data.P2(row, col) = value;
    }
  }

  std::getline(ifs, line);
  ss.clear();
  ss.str(line.substr(4));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      ss >> value;
      calibration_data.P3(row, col) = value;
    }
  }

  std::getline(ifs, line);
  ss.clear();
  ss.str(line.substr(9));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      ss >> value;
      calibration_data.R0_rect(row, col) = value;
    }
  }

  std::getline(ifs, line);
  ss.clear();
  ss.str(line.substr(16));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      ss >> value;
      calibration_data.velo_to_cam(row, col) = value;
    }
  }

  std::getline(ifs, line);
  ss.clear();
  ss.str(line.substr(16));
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      ss >> value;
      calibration_data.imu_to_velo(row, col) = value;
    }
  }
  ifs.close();
  return 0;
}

bool Dataloader::load_kitti_label(
  const std::string& label_path,
  std::vector<kitti::Label>& labels)
{
  std::ifstream ifs(label_path);
  std::string line;
  if (!ifs.is_open())
  {
    std::cout << "cannot open label_path : " << label_path << std::endl;
    return -1;
  }
  while (std::getline(ifs, line))
  {
    kitti::Label label;
    std::stringstream ss(line);
    ss >> label.type >> label.truncated >> label.occluded >> label.alpha >>
      label.left >> label.top >> label.right >> label.bottom >> label.height >>
      label.width >> label.length >> label.loc_x >> label.loc_y >>
      label.loc_z >> label.rot_y >> label.score;
    if (label.score > 0.7)
    {
      labels.push_back(label);
    }
  }
  ifs.close();
  return 0;
}

bool Dataloader::load_kitti_image(const std::string& image_path, cv::Mat& image)
{
  image = cv::imread(image_path, cv::IMREAD_ANYCOLOR);
  if (image.empty())
  {
    std::cout << "cannot load image from image_path : !" << image_path
              << std::endl;
    return -1;
  }
  return 0;
}

kitti::Data Dataloader::get_kitti_data(const uint frame_count)
{
  kitti::Data kitti_data;
  std::string str_frame_count = std::to_string(frame_count);
  std::string file_name =
    std::string(string_length_ - str_frame_count.length(), '0') +
    str_frame_count;
  std::string calibration_file_path =
    kitti_root_path_ + calibration_path_ + file_name + ".txt";
  std::string image_file_path =
    kitti_root_path_ + image_path_ + file_name + ".png";
  std::string label_file_path =
    kitti_root_path_ + label_path_ + file_name + ".txt";
  load_kitti_calibration(calibration_file_path, kitti_data.calibration);
  load_kitti_image(image_file_path, kitti_data.image);
  load_kitti_label(label_file_path, kitti_data.labels);
  return kitti_data;
}

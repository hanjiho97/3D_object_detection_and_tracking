#include "object_tracking/Measurement.h"

Measurement::Measurement()
{
  R_ = Eigen::MatrixXd::Zero(3, 3);
  double sigma_lidar_x = 0.1;
  double sigma_lidar_y = 0.1;
  double sigma_lidar_z = 0.1;
  R_(0, 0) = sigma_lidar_x * sigma_lidar_x;
  R_(1, 1) = sigma_lidar_y * sigma_lidar_y;
  R_(2, 2) = sigma_lidar_z * sigma_lidar_z;

  cam_to_veh_ = Eigen::MatrixXd::Identity(4, 4);
  veh_to_cam_ = cam_to_veh_.inverse();

  t_ = 1.0;
  z_ = Eigen::VectorXd(3);
  z_ << 1.0, 2.0, 3.0;
  type_ = "Car";
  width_ = 2.0;
  height_ = 2.0;
  length_ = 2.0;
  rot_y_ = 0.0;
}

Measurement::Measurement(
  uint frame_count,
  uint detection_count,
  const kitti::Data &kitti_data)
{
  R_ = Eigen::MatrixXd::Zero(3, 3);
  double sigma_lidar_x = 0.1;
  double sigma_lidar_y = 0.1;
  double sigma_lidar_z = 0.1;
  R_(0, 0) = sigma_lidar_x * sigma_lidar_x;
  R_(1, 1) = sigma_lidar_y * sigma_lidar_y;
  R_(2, 2) = sigma_lidar_z * sigma_lidar_z;

  veh_to_cam_ = Eigen::MatrixXd::Identity(4, 4);
  veh_to_cam_.block<3, 4>(0, 0) = kitti_data.calibration.velo_to_cam;
  cam_to_veh_ = veh_to_cam_.inverse();

  t_ = static_cast<double>(frame_count) * 0.1;
  z_ = Eigen::VectorXd(3);
  z_ << kitti_data.labels[detection_count].loc_x,
    kitti_data.labels[detection_count].loc_y,
    kitti_data.labels[detection_count].loc_z;
  type_ = kitti_data.labels[detection_count].type;
  width_ = kitti_data.labels[detection_count].width;
  height_ = kitti_data.labels[detection_count].height;
  length_ = kitti_data.labels[detection_count].length;
  rot_y_ = kitti_data.labels[detection_count].rot_y;
}

Measurement::~Measurement() {}

Eigen::VectorXd Measurement::get_z() const
{
  return z_;
}

Eigen::MatrixXd Measurement::get_R() const
{
  return R_;
}

Eigen::VectorXd Measurement::get_hx(const Eigen::VectorXd &x) const
{
  Eigen::VectorXd pos_veh = Eigen::VectorXd::Ones(4);
  pos_veh.block<3, 1>(0, 0) = x.block<3, 1>(0, 0);
  Eigen::VectorXd pos_cam = veh_to_cam_ * pos_veh;
  return pos_cam.block<3, 1>(0, 0);
}

Eigen::MatrixXd Measurement::get_H(const Eigen::VectorXd &x) const
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
  H.block<3, 3>(0, 0) = veh_to_cam_.block<3, 3>(0, 0);
  return H;
}

double Measurement::get_t() const
{
  return t_;
}

Attributes Measurement::get_attributes() const
{
  Attributes ret;
  ret.loc_x = z_(0);
  ret.loc_y = z_(1);
  ret.loc_z = z_(2);
  ret.height = height_;
  ret.length = length_;
  ret.width = width_;
  ret.rot_y = rot_y_;
  return ret;
}


Eigen::MatrixXd Measurement::get_cam_to_veh() const
{
  return cam_to_veh_;
}

Eigen::MatrixXd Measurement::get_veh_to_cam() const
{
  return veh_to_cam_;
}

void Measurement::kitti_to_measurement_list(uint frame_count, const kitti::Data& kitti_data, std::vector<Measurement>& meas_list)
{
  meas_list.clear();
  meas_list.reserve(kitti_data.labels.size());
  for(uint label_num = 0; label_num < kitti_data.labels.size(); ++label_num)
    {
      Measurement measurement = Measurement(frame_count, label_num, kitti_data);
      meas_list.push_back(measurement);
    }
}

void Measurement::print() const
{
  std::cout << "Measurement print ------------------------------" << std::endl;
  std::cout << "z_ = " << std::endl << z_ << std::endl;
  std::cout << "cam_to_veh_ = " << std::endl << cam_to_veh_ << std::endl;
  std::cout << "veh_to_cam_ = " << std::endl << veh_to_cam_ << std::endl;
  std::cout << "R_ = " << std::endl << R_ << std::endl;
  std::cout << "t_ = " << std::endl << t_ << std::endl;
  std::cout << "class_num_ = " << std::endl << type_ << std::endl;
  std::cout << "width_ = " << std::endl << width_ << std::endl;
  std::cout << "height_ = " << std::endl << height_ << std::endl;
  std::cout << "length_ = " << std::endl << length_ << std::endl;
  std::cout << "rot_y_ = " << std::endl << rot_y_ << std::endl;
}

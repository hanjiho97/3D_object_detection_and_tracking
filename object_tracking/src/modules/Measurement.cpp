#include "modules/Measurement.h"

Measurement::Measurement()
{
  R_ = Eigen::MatrixXd::Zero(3, 3);
  double sigma_lidar_x = 0.1;
  double sigma_lidar_y = 0.1;
  double sigma_lidar_z = 0.1;
  R_(0, 0) = sigma_lidar_x * sigma_lidar_x;
  R_(1, 1) = sigma_lidar_y * sigma_lidar_y;
  R_(2, 2) = sigma_lidar_z * sigma_lidar_z;

  // TODO : inference 결과로부터 멤버변수들 값 할당.
  // 현재는 dummy임
  cam_to_veh_ = Eigen::MatrixXd::Identity(4, 4);
  veh_to_cam_ = cam_to_veh_.inverse();

  t_ = 1.0;
  z_ = Eigen::VectorXd(3);
  z_ << 1.0, 2.0, 3.0;
  type_ = "Car";
  width_ = 2.0;
  height_ = 2.0;
  length_ = 2.0;
  yaw_ = 0.0;
}

Measurement::Measurement(
  int frame_cnt,
  const Kitti_Object &obj,
  const Kitti_Calib &calib)
{
  R_ = Eigen::MatrixXd::Zero(3, 3);
  double sigma_lidar_x = 0.1;
  double sigma_lidar_y = 0.1;
  double sigma_lidar_z = 0.1;
  R_(0, 0) = sigma_lidar_x * sigma_lidar_x;
  R_(1, 1) = sigma_lidar_y * sigma_lidar_y;
  R_(2, 2) = sigma_lidar_z * sigma_lidar_z;

  veh_to_cam_ = Eigen::MatrixXd::Identity(4, 4);
  veh_to_cam_.block<3, 4>(0, 0) = calib.velo_to_cam;
  cam_to_veh_ = veh_to_cam_.inverse();

  t_ = static_cast<double>(frame_cnt) * 0.1;  // 수정할 것
  z_ = Eigen::VectorXd(3);
  z_ << obj.loc_x, obj.loc_y, obj.loc_z;
  type_ = obj.type;
  width_ = obj.width;
  height_ = obj.height;
  length_ = obj.length;
  yaw_ = obj.rot_y;
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


Eigen::MatrixXd Measurement::get_cam_to_veh() const
{
  return cam_to_veh_;
}

void Measurement::print()
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
  std::cout << "yaw_ = " << std::endl << yaw_ << std::endl;
}

void load_kitti_label(
  const std::string &label_path,
  std::vector<Kitti_Object> &objs)
{
  std::ifstream ifs(label_path);
  std::string line;
  if (ifs.is_open())
  {
    while (std::getline(ifs, line))
    {
      Kitti_Object obj;
      std::stringstream ss(line);
      ss >> obj.type >> obj.truncated >> obj.occluded >> obj.alpha >>
        obj.left >> obj.top >> obj.right >> obj.bottom >> obj.height >>
        obj.width >> obj.length >> obj.loc_x >> obj.loc_y >> obj.loc_z >>
        obj.rot_y >> obj.score;
      objs.push_back(obj);
    }
    ifs.close();
  }
  else
  {
    std::cout << "cannot open label_path : " << label_path << std::endl;
  }
}

Kitti_Calib load_kitti_calib(const std::string &calib_path)
{
  std::ifstream ifs(calib_path);
  std::string line;
  Kitti_Calib calib;
  double value;
  if (ifs.is_open())
  {
    std::getline(ifs, line);
    std::stringstream ss(line.substr(4));
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 4; ++col)
      {
        ss >> value;
        calib.P0(row, col) = value;
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
        calib.P1(row, col) = value;
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
        calib.P2(row, col) = value;
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
        calib.P3(row, col) = value;
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
        calib.R0_rect(row, col) = value;
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
        calib.velo_to_cam(row, col) = value;
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
        calib.imu_to_velo(row, col) = value;
      }
    }
    ifs.close();
  }
  else
  {
    std::cout << "cannot open calib_path : " << calib_path << std::endl;
  }


  // std::cout << "P0" << std::endl << calib.P0 << std::endl;
  // std::cout << "P1" << std::endl << calib.P1 << std::endl;
  // std::cout << "P2" << std::endl << calib.P2 << std::endl;
  // std::cout << "P3" << std::endl << calib.P3 << std::endl;
  // std::cout << "R0" << std::endl << calib.R0_rect << std::endl;
  // std::cout << "velo2cam" << std::endl << calib.velo_to_cam << std::endl;
  // std::cout << "imu2velo" << std::endl << calib.imu_to_velo << std::endl;

  return calib;
}
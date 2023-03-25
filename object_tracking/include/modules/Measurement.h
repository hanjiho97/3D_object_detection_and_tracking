#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

struct Kitti_Object
{
    std::string type;
    double truncated;
    double occluded;
    double alpha;
    double left, top, right, bottom; // 2D bbox
    double height, width, length;
    double loc_x, loc_y, loc_z; // camera coordinates
    double rot_y;
    double score;
};

struct Kitti_Calib
{
    Eigen::Matrix<double, 3, 4> P0, P1, P2, P3;
    Eigen::Matrix<double, 3, 3> R0_rect;
    Eigen::Matrix<double, 3, 4> velo_to_cam;
    Eigen::Matrix<double, 3, 4> imu_to_velo;
};

void load_kitti_label(const std::string &label_path, std::vector<Kitti_Object> &objs);
Kitti_Calib load_kitti_calib(const std::string &calib_path);

struct Attributes
{
    double height, width, length;
    double rot_y;
};

class Measurement
{
public:
    Measurement();
    Measurement(int frame_cnt, const Kitti_Object &obj, const Kitti_Calib &calib);
    virtual ~Measurement();

    Eigen::VectorXd get_z() const;
    Eigen::MatrixXd get_R() const;
    Eigen::VectorXd get_hx(const Eigen::VectorXd &x) const;
    Eigen::MatrixXd get_H(const Eigen::VectorXd &x) const;
    Eigen::MatrixXd get_cam_to_veh() const;
    double get_t() const;
    Attributes get_attributes() const;

    void print();

private:
    Eigen::VectorXd z_;
    Eigen::MatrixXd cam_to_veh_;
    Eigen::MatrixXd veh_to_cam_;
    Eigen::MatrixXd R_;

    double t_;

    std::string type_;
    double width_;
    double length_;
    double height_;
    double rot_y_;
};

#endif // MEASUREMENT_H_
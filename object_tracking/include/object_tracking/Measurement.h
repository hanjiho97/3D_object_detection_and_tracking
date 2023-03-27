#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "object_tracking/Dataloader.h"

struct Attributes
{
  double height, width, length;
  double rot_y;
};

class Measurement
{
public:
  Measurement();
  Measurement(uint frame_count,
  uint detection_count,
  const kitti::Data &kitti_data);
  virtual ~Measurement();

  Eigen::VectorXd get_z() const;
  Eigen::MatrixXd get_R() const;
  Eigen::VectorXd get_hx(const Eigen::VectorXd &x) const;
  Eigen::MatrixXd get_H(const Eigen::VectorXd &x) const;
  Eigen::MatrixXd get_cam_to_veh() const;
  double get_t() const;
  Attributes get_attributes() const;

  void print() const;

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

#endif  // MEASUREMENT_H_
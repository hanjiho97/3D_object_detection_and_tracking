#ifndef TRACKER_H_
#define TRACKER_H_

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Association.h"

class Track
{
public:
  Track(const Measurement& meas, uint id);
  virtual ~Track();

  void update_attributes(const Measurement& meas);

  Eigen::VectorXd get_x();
  Eigen::MatrixXd get_P();
  double get_t();

  void set_x(const Eigen::VectorXd& x);
  void set_P(const Eigen::MatrixXd& P);

  void print() const;


private:
  uint id_;
  double t_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  Eigen::MatrixXd rot_cam_to_veh_;

  double score_;
  uint state_;  // 0 : init, 1 : tentative, 2 : confirmed

  Attributes attributes_;
};

class TrackManager
{
public:
  TrackManager();
  virtual ~TrackManager();
  void add_new_track(const Measurement& meas);
  void delete_track();
private:
  uint current_num_tracks_;
  std::vector<Track> track_list_;
  uint last_id_;
  

};

#endif  // TRACKER_H_
#ifndef TRACKER_H_
#define TRACKER_H_

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "object_tracking/type.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Association.h"

class Track
{
public:
  Track(const Measurement& meas, uint id);
  virtual ~Track();

  void update_attributes(const Measurement& meas);

  const Eigen::VectorXd& get_x() const;
  const Eigen::MatrixXd& get_P() const;
  double get_t() const;
  uint get_id() const;

  void set_x(const Eigen::VectorXd& x);
  void set_P(const Eigen::MatrixXd& P);

  void print() const;

  const Attributes& get_attributes() const;

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
  void delete_track(uint id);

  const std::vector<Track>& get_track_list() const;
  
  void handle_updated_track(uint id);

private:
  uint current_num_tracks_;
  uint last_id_;
  std::vector<Track> track_list_;

};

#endif  // TRACKER_H_
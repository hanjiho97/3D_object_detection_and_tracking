#ifndef TRACKER_H_
#define TRACKER_H_

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "Eigen/Dense"

#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/Association.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"

class EKF;

class Track
{
public:
  Track(const Measurement& meas, uint id);
  virtual ~Track();

  void update_attributes(const Measurement& meas);
  void update_location();

  const Eigen::VectorXd& get_x() const;
  const Eigen::MatrixXd& get_P() const;
  double get_t() const;
  uint get_id() const;
  double get_score() const;
  uint get_state() const;

  void set_t(double t);
  void set_x(const Eigen::VectorXd& x);
  void set_P(const Eigen::MatrixXd& P);
  void set_score(double score);
  void set_state(uint state);

  void print() const;

  const Attributes& get_attributes() const;

private:
  uint id_;
  double t_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  Eigen::MatrixXd veh_to_cam_;
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

  const std::map<uint, Track>& get_track_list() const;
  std::map<uint, Attributes> get_attributes();

  void manage_tracks(
    std::vector<uint> unassigned_track_ids,
    std::vector<uint> unassigned_meas_idxs,
    const std::vector<Measurement>& meas_list);
  void predict_tracks(uint frame_count, EKF& ekf);
  void update_track(uint id, const Measurement& meas, EKF& ekf);

  void print();
private:
  uint current_num_tracks_;
  uint last_id_;
  std::map<uint, Track> track_list_;
};

#endif  // TRACKER_H_
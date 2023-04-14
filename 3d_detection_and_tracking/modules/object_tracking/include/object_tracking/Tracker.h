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
  Track(const Measurement& meas, uint32_t id);
  virtual ~Track();

  const Eigen::VectorXd& get_x() const;
  Eigen::VectorXd get_location() const;
  const Eigen::MatrixXd& get_P() const;
  double get_t() const;
  uint32_t get_id() const;
  double get_score() const;
  uint8_t get_state() const;

  void set_t(double t);
  void set_x(const Eigen::VectorXd& x);
  void set_P(const Eigen::MatrixXd& P);
  void set_score(double score);
  void set_state(uint8_t state);

  void print() const;

private:
  uint32_t id_;
  double t_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  Eigen::MatrixXd veh_to_cam_;
  Eigen::MatrixXd rot_cam_to_veh_;

  double score_;
  uint8_t state_;  // 0 : init, 1 : tentative, 2 : confirmed
};

class TrackManager
{
public:
  TrackManager();
  virtual ~TrackManager();
  void add_new_track(const Measurement& meas);
  void delete_track(uint32_t id);
  const std::map<uint32_t, Track>& get_track_list() const;
  const std::map<uint32_t, Attributes>& get_attributes() const;

  void manage_tracks(
    std::vector<uint32_t> unassigned_track_ids,
    std::vector<uint32_t> unassigned_meas_idxs,
    const std::vector<Measurement>& meas_list);
  void predict_tracks(uint16_t frame_count, EKF& ekf);
  void update_track(uint32_t id, const Measurement& meas, EKF& ekf);

  void print();

private:
  void update_location(uint32_t id);
  void update_attributes(uint32_t id, const Measurement& meas);

  uint32_t current_num_tracks_;
  uint32_t last_id_;
  std::map<uint32_t, Track> track_list_;
  std::map<uint32_t, Attributes> attributes_list_;
};

#endif  // TRACKER_H_
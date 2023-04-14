#ifndef ASSOCIATION_H_
#define ASSOCIATION_H_

#include <cstdint>
#include <iostream>
#include <limits>
#include <tuple>

#include "Eigen/Dense"
#include "spdlog/spdlog.h"

#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"

class EKF;
class Track;
class TrackManager;

class Association
{
public:
  Association();
  virtual ~Association();

  void associate(
    TrackManager& track_manager,
    const std::vector<Measurement>& meas_list,
    const EKF& ekf);
  void associate_and_update(
    TrackManager& track_manager,
    const std::vector<Measurement>& meas_list,
    EKF& ekf);

private:
  double MHD(const Track& track, const Measurement& meas, const EKF& ekf);
  std::tuple<int32_t, uint32_t, uint32_t> get_closest_pair();

  double limit_distance_;
  Eigen::MatrixXd association_matrix;
  std::vector<uint32_t> unassigned_track_ids;
  std::vector<uint32_t> unassigned_meas_indexes;
};

#endif  // ASSOCIATION_H_
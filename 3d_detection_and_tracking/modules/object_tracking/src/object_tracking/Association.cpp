#include "object_tracking/Association.h"

Association::Association()
{
  limit_distance_ = 10.597;  // chi square : df = 3 - 1, 0.005
  // limit_distance_ = 7.879; // chi square : df = 2 - 1, 0.005 for camera
  // measurement
}

Association::~Association() {}

void Association::associate(
  TrackManager& track_manager,
  const std::vector<Measurement>& meas_list,
  const EKF& ekf)
{
  const std::map<uint, Track>& track_list = track_manager.get_track_list();
  uint N = track_list.size();
  uint M = meas_list.size();
  unassigned_track_ids.clear();
  unassigned_meas_indexes.clear();
  unassigned_track_ids.reserve(N);
  unassigned_meas_indexes.reserve(M);
 
  association_matrix =
    Eigen::MatrixXd::Constant(N, M, std::numeric_limits<double>::max());

  for (uint i = 0; i < M; ++i)
  {
    unassigned_meas_indexes.push_back(i);
  }

  uint row(0), col(0);
  for (const auto& track_pair : track_list)
  {
    col = 0;
    unassigned_track_ids.push_back(track_pair.first);
    for (const auto& meas : meas_list)
    {
      double mhd = MHD(track_pair.second, meas, ekf);
      if (mhd <= limit_distance_)
      {
        association_matrix(row, col) = mhd;
      }
      ++col;
    }
    ++row;
  }
}

double
Association::MHD(const Track& track, const Measurement& meas, const EKF& ekf)
{
  Eigen::VectorXd y = ekf.get_y(track, meas);
  Eigen::MatrixXd S = ekf.get_S(track, meas);
  Eigen::VectorXd x = track.get_x();
  Eigen::MatrixXd H = meas.get_H(x);
  // square of mahalanobis distance
  return (y.transpose() * S.inverse() * y)(0, 0);
}

std::tuple<int, uint, uint> Association::get_closest_pair()
{
  uint i, track_id, j, meas_index;
  double min_val = association_matrix.minCoeff(&i, &j);
  if (std::abs(std::numeric_limits<double>::max() - min_val) < 1.0)
  {
    return std::make_tuple(-1, track_id, meas_index);
  }
  track_id = unassigned_track_ids[i];
  meas_index = unassigned_meas_indexes[j];

  unassigned_track_ids.erase(unassigned_track_ids.begin() + i);
  unassigned_meas_indexes.erase(unassigned_meas_indexes.begin() + j);

  // remove row
  Eigen::MatrixXd mat_1(
    association_matrix.rows() - 1, association_matrix.cols());
  mat_1 << association_matrix.topRows(i),
    association_matrix.bottomRows(association_matrix.rows() - (i + 1));

  // remove col
  Eigen::MatrixXd mat_2(mat_1.rows(), mat_1.cols() - 1);
  mat_2 << mat_1.leftCols(j), mat_1.rightCols(mat_1.cols() - (j + 1));
  association_matrix = mat_2;

  return std::make_tuple(0, track_id, meas_index);
}

void Association::associate_and_update(
  TrackManager& track_manager,
  const std::vector<Measurement>& meas_list,
  EKF& ekf)
{
  associate(track_manager, meas_list, ekf);
  while (association_matrix.rows() > 0 && association_matrix.cols() > 0)
  {
    int ret;
    uint track_id, meas_index;
    std::tie(ret, track_id, meas_index) = get_closest_pair();
    if (ret == -1)
    {
      break;
    }

    track_manager.update_track(track_id, meas_list[meas_index], ekf);
  }

  track_manager.manage_tracks(unassigned_track_ids, unassigned_meas_indexes, meas_list);
}

#include "object_tracking/Tracker.h"

Track::Track(const Measurement& meas, uint id)
{
  id_ = id;

  Eigen::VectorXd z = meas.get_z();
  Eigen::VectorXd pos_cam = Eigen::VectorXd::Ones(4);
  pos_cam.block<3, 1>(0, 0) = z;
  Eigen::MatrixXd cam_to_veh = meas.get_cam_to_veh();
  Eigen::VectorXd pos_veh = cam_to_veh * pos_cam;
  rot_cam_to_veh_ = cam_to_veh.block<3, 3>(0, 0);
  veh_to_cam_ = meas.get_veh_to_cam();
  x_ = Eigen::VectorXd::Zero(6);
  x_.block<3, 1>(0, 0) = pos_veh.block<3, 1>(0, 0);

  P_ = Eigen::MatrixXd::Zero(6, 6);
  P_.block<3, 3>(0, 0) =
    rot_cam_to_veh_ * meas.get_R() * rot_cam_to_veh_.transpose();

  double sigma_p44 = 50.0;
  double sigma_p55 = 50.0;
  double sigma_p66 = 5.0;

  P_(3, 3) = sigma_p44 * sigma_p44;
  P_(4, 4) = sigma_p55 * sigma_p55;
  P_(5, 5) = sigma_p66 * sigma_p66;

  t_ = meas.get_t();
  attributes_ = meas.get_attributes();
  state_ = 0;
  score_ = 1.0 / 6.0;
}

Track::~Track() {}

const Eigen::VectorXd& Track::get_x() const
{
  return x_;
}
const Eigen::MatrixXd& Track::get_P() const
{
  return P_;
}
double Track::get_t() const
{
  return t_;
}

uint Track::get_id() const
{
  return id_;
}

double Track::get_score() const
{
  return score_;
}
uint Track::get_state() const
{
  return state_;
}

void Track::set_score(double score)
{
  score_ = score;
}
void Track::set_state(uint state)
{
  state_ = state;
}

void Track::set_t(double t)
{
  t_ = t;
}

const Attributes& Track::get_attributes() const
{
  return attributes_;
}

void Track::update_location()
{
  Eigen::VectorXd pos_veh = Eigen::VectorXd::Ones(4);
  pos_veh.block<3, 1>(0, 0) = x_.block<3, 1>(0, 0);
  Eigen::VectorXd pos_cam = veh_to_cam_ * pos_veh;

  attributes_.loc_x = pos_cam(0, 0);
  attributes_.loc_y = pos_cam(1, 0);
  attributes_.loc_z = pos_cam(2, 0);
}

void Track::update_attributes(const Measurement& meas)
{
  Attributes meas_attributes = meas.get_attributes();
  attributes_.height = 0.9 * attributes_.height + 0.1 * meas_attributes.height;
  attributes_.width = 0.9 * attributes_.width + 0.1 * meas_attributes.width;
  attributes_.length = 0.9 * attributes_.length + 0.1 * meas_attributes.length;
  attributes_.rot_y = meas_attributes.rot_y;
}

void Track::set_x(const Eigen::VectorXd& x)
{
  x_ = x;
}

void Track::set_P(const Eigen::MatrixXd& P)
{
  P_ = P;
}

void Track::print() const
{
  std::cout << "Track print ------------------------------" << std::endl;
  std::cout << "id_ = " << std::endl << id_ << std::endl;
  std::cout << "t_ = " << std::endl << t_ << std::endl;
  std::cout << "x_ = " << std::endl << x_ << std::endl;
  std::cout << "P_ = " << std::endl << P_ << std::endl;
  std::cout << "width_ = " << std::endl << attributes_.width << std::endl;
  std::cout << "height_ = " << std::endl << attributes_.height << std::endl;
  std::cout << "length_ = " << std::endl << attributes_.length << std::endl;
  std::cout << "yaw_ = " << std::endl << attributes_.rot_y << std::endl;
}

// TrackManager ---------------------------------------------------------

TrackManager::TrackManager() 
{
  last_id_ = 0;
  current_num_tracks_ = 0;
}
TrackManager::~TrackManager() {}

void TrackManager::add_new_track(const Measurement& meas)
{
  Track track(meas, ++last_id_);
  track_list_.insert({track.get_id(), track});
  ++current_num_tracks_;
}

const std::map<uint, Track>& TrackManager::get_track_list() const
{
  return track_list_;
}

std::map<uint, Attributes> TrackManager::get_attributes()
{
  std::map<uint, Attributes> ret;
  for(const auto& track_pair : track_list_)
  {
    ret[track_pair.first] = track_pair.second.get_attributes();
  }
  return ret;
}

void TrackManager::manage_tracks(
  std::vector<uint> unassigned_track_ids,
  std::vector<uint> unassigned_meas_indexes,
  const std::vector<Measurement>& meas_list)
{
  for (uint id : unassigned_track_ids)
  {
    if (track_list_.find(id) == track_list_.end())
    {
      continue;
    }
    double current_score = track_list_.at(id).get_score();
    track_list_.at(id).set_score(current_score - 1.0 / 6.0);
  }

  std::vector<uint> track_id_to_delete;

  for (auto iter = track_list_.begin(); iter != track_list_.end(); ++iter)
  {
    uint state = iter->second.get_state();
    double score = iter->second.get_score();
    Eigen::MatrixXd P = iter->second.get_P();
    if (
      (state == 2 && score <= 0.6) || (P(0, 0) > 9 || P(1, 1) > 9) ||
      score < 0.05)
    {
      track_id_to_delete.push_back(iter->first);
    }
  }

  for (uint id : track_id_to_delete)
  {
    track_list_.erase(id);
  }

  for (uint index : unassigned_meas_indexes)
  {
    add_new_track(meas_list[index]);
  }
}

void TrackManager::predict_tracks(uint frame_count, EKF& ekf)
{
  for(auto& track_pair : track_list_)
  {
    ekf.predict(frame_count, track_pair.second);
  }
}

void TrackManager::update_track(uint id, const Measurement& meas, EKF& ekf)
{
  ekf.update(track_list_.at(id), meas);
  if (track_list_.find(id) == track_list_.end())
  {
    return;
  }
  double current_score = track_list_.at(id).get_score();

  current_score = std::min(1.0, current_score + 1.0 / 6.0);
  track_list_.at(id).set_score(current_score);
  if (current_score >= 0.8)
  {
    track_list_.at(id).set_state(2);
  }
  else
  {
    track_list_.at(id).set_state(1);
  }
}

void TrackManager::print()
{
  for(const auto& track_pair : track_list_)
  {
    track_pair.second.print();
  }
}
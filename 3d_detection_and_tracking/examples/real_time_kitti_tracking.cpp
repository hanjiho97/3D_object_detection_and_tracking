#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>

#include "3d_detection_and_tracking/Type.h"
#include "object_tracking/Association.h"
#include "object_tracking/Dataloader.h"
#include "object_tracking/EKF.h"
#include "object_tracking/Measurement.h"
#include "object_tracking/Tracker.h"
#include "result_viewer/Projection.h"
#include "result_viewer/Viewer.h"

int read_pipe()
{
  int fifo_d;
  boost::asio::io_service io_service;
  std::vector<uint8_t> buffer(6);

  fifo_d = open(
    "/home/haeryong/Documents/virconv_mount/output/frame.txt",
    O_RDONLY | O_NONBLOCK);
  boost::asio::posix::stream_descriptor fifo(io_service, fifo_d);
  boost::asio::async_read(
    fifo,
    boost::asio::buffer(buffer),
    [&](const boost::system::error_code& ec, std::size_t size) {});
  io_service.run();
  close(fifo_d);

  std::string str;
  str.reserve(6);
  for (auto x : buffer)
  {
    str.push_back(char(x));
  }
  std::cout << "str : " << str << std::endl;
  if (!isdigit(str[0]))
  {
    return -1;
  }
  return std::stoi(str);
}

int main()
{
  kitti::Data kitti_data;
  Dataloader data_loader = Dataloader();
  TrackManager track_manager = TrackManager();
  Association association = Association();
  EKF ekf = EKF();
  Viewer viewer = Viewer();
  std::vector<Measurement> meas_list;
  bool show_bbox_3D=true;
  bool showing_head=true;
  bool showing_id=true;
  bool showing_topview=true;

  int frame_count = -1;
  bool filter_update = true;

  while (read_pipe() != 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  while (true)
  {
    filter_update = true;
    int current_frame_count = read_pipe();
    if (current_frame_count == -1 || current_frame_count == frame_count)
    {
      filter_update = false;
    }
    else
    {
      frame_count = current_frame_count;
    }

    if (filter_update == true)
    {
      kitti_data = data_loader.get_kitti_data(frame_count);
      viewer.read_P2_matrix(kitti_data.calibration.P2);
      viewer.add_image(kitti_data.image);
      meas_list.clear();
      meas_list.reserve(kitti_data.labels.size());
      for (uint label_num = 0; label_num < kitti_data.labels.size();
           ++label_num)
      {
        Measurement meas = Measurement(frame_count, label_num, kitti_data);
        meas_list.push_back(meas);
      }

      track_manager.predict_tracks(frame_count, ekf);
      association.associate_and_update(track_manager, meas_list, ekf);
    }

    std::map<uint, Attributes> attributes_list = std::move(track_manager.get_attributes());
    for(auto& attr_pair : attributes_list)
    {
      std::cout << "attr key : " << attr_pair.first << std::endl; 
    }
    std::map<uint, Track> track_list = track_manager.get_track_list();
    for (auto& track_pair : track_list)
    {
      if (track_pair.second.get_state() == 2)
      {
        viewer.add_3d_bbox(track_pair.first, track_pair.second.get_attributes());
        viewer.draw(
        show_bbox_3D, 
        showing_head, 
        showing_id,
        showing_topview);
      }
    }
    viewer.show_result();
  }
  return 0;
}

/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#if ROS_AVAILABLE == 1
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#endif

#include "core/VioManagerOptions.h"
#include "sim/SimulatorUnderwater.h"

using namespace ov_msckf;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  // Launch our ros node
  ros::init(argc, argv, "test_sim_meas");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);

  // setup for rosbag
  bool save_bag;
  std::string bag_path;
  nh->param<bool>("save_bag", save_bag, false);
  nh->param<std::string>("filepath_bag", bag_path, "/home/lin/Desktop/sim.bag");
  rosbag::Bag bag;
  if(save_bag) {
    bag.open(bag_path, rosbag::bagmode::Write);
  }
  std::cout<<"file path: "<< bag_path<<std::endl;

#endif

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#endif

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create the simulator
  VioManagerOptions params;
  params.print_and_load(parser);
  params.print_and_load_simulation(parser);
  SimulatorUnderwater sim(params);

  // Continue to simulate until we have processed all the measurements
  signal(SIGINT, signal_callback_handler);
  while (sim.ok()) {

    // ground truth states:
    Eigen::Matrix<double, 17, 1> state_gt;
    double sim_time = sim.current_timestamp();
    if(sim.get_state(sim_time, state_gt) && save_bag) {
      // save q, p
      geometry_msgs::PoseStamped poseIinG;
      poseIinG.header.stamp = ros::Time(state_gt(0, 0));
      poseIinG.header.frame_id = "global";
      poseIinG.pose.orientation.x = state_gt(1, 0);
      poseIinG.pose.orientation.y = state_gt(2, 0);
      poseIinG.pose.orientation.z = state_gt(3, 0);
      poseIinG.pose.orientation.w = state_gt(4, 0);
      poseIinG.pose.position.x = state_gt(5, 0);
      poseIinG.pose.position.y = state_gt(6, 0);
      poseIinG.pose.position.z = state_gt(7, 0);
      bag.write("/ov_msckf/state_gt/pose", ros::Time(state_gt(0, 0)), poseIinG);

      // save v
      geometry_msgs::TwistStamped velocityIinG;
      velocityIinG.header.stamp = ros::Time(state_gt(0, 0));
      velocityIinG.header.frame_id = "global";
      velocityIinG.twist.linear.x = state_gt(8, 0);      
      velocityIinG.twist.linear.y = state_gt(9, 0);      
      velocityIinG.twist.linear.z = state_gt(10, 0);  
      bag.write("/ov_msckf/state_gt/velocity", ros::Time(state_gt(0, 0)), velocityIinG);

      // save gyro bias 
      geometry_msgs::Vector3Stamped bg;
      bg.header.stamp = ros::Time(state_gt(0, 0));
      bg.header.frame_id = "imu";
      bg.vector.x = state_gt(11, 0);
      bg.vector.y = state_gt(12, 0);
      bg.vector.z = state_gt(13, 0);
      bag.write("/ov_msckf/state_gt/bg", ros::Time(state_gt(0, 0)), bg);

      // save accl bias 
      geometry_msgs::Vector3Stamped ba;
      ba.header.stamp = ros::Time(state_gt(0, 0));
      ba.header.frame_id = "imu";
      ba.vector.x = state_gt(14, 0);
      ba.vector.y = state_gt(15, 0);
      ba.vector.z = state_gt(16, 0);
      bag.write("/ov_msckf/state_gt/ba", ros::Time(state_gt(0, 0)), ba);      
    }

    // IMU: get the next simulated IMU measurement if we have it
    double time_imu;
    Eigen::Vector3d wm, am;
    bool hasimu = sim.get_next_imu(time_imu, wm, am);
    if (hasimu) {
      PRINT_DEBUG("new imu measurement = %0.15g | w = %0.3g | a = %0.3g\n", time_imu, wm.norm(), am.norm());

      // save to rosbag
      sensor_msgs::Imu msg_imu;
      msg_imu.header.stamp = ros::Time(time_imu);
      msg_imu.header.frame_id = "imu";
      msg_imu.linear_acceleration.x = am(0);
      msg_imu.linear_acceleration.y = am(1);
      msg_imu.linear_acceleration.z = am(2);
      msg_imu.angular_velocity.x = wm(0);
      msg_imu.angular_velocity.y = wm(1);
      msg_imu.angular_velocity.z = wm(2);
      bag.write("/ov_msckf/sim/imu", ros::Time(time_imu), msg_imu);
    }

    // CAM: get the next simulated camera uv measurements if we have them
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    bool hascam = sim.get_next_cam(time_cam, camids, feats);
    if (hascam) {
      PRINT_DEBUG("new cam measurement = %0.15g | %u cameras | uvs(0) = %u \n", time_cam, camids.size(), feats.at(0).size());

      // save to bag
      for(size_t i = 0; i <feats.size(); i++) {

        sensor_msgs::PointCloud::Ptr cam_uvs(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 cam_ids;

        for(const auto& feat : feats.at(i)) {
          geometry_msgs::Point32 measurement;
          // u
          measurement.x = feat.second(0);
          // v
          measurement.y = feat.second(1);
          // 1
          measurement.z = 1.0;

          cam_uvs->points.push_back(measurement);
          cam_ids.values.push_back(feat.first);
        }

        std::string ros_topic = "/ov_msckf/sim/cam_uvs" + std::to_string(i);
        cam_uvs->channels.push_back(cam_ids);
        bag.write(ros_topic, ros::Time(time_cam), cam_uvs);
      }



    }
  }

#ifdef ROS_AVAILABLE
  bag.close();
#endif

  // Done!
  return EXIT_SUCCESS;
};
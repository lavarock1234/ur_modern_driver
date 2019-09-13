/*
 * Copyright 2019 Dennis Ding (tracker)
 *
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"

class Tracker : public Service {
private:
  URCommander &commander_;
  ros::NodeHandle nh_;
  ros::Subscriber point_sub_;
  RobotState state_;
  ros::Timer timer_;
  ros::Time last_track_;

public:
  Tracker(URCommander &commander);
  void onRobotStateChange(RobotState state);
  virtual ~Tracker(){};

private:
  void point_cb(trajectory_msgs::JointTrajectoryPointConstPtr point);
  void stop();
};

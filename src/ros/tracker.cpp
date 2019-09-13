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

#include "ur_modern_driver/ros/tracker.h"
#include <endian.h>
#include <ros/ros.h>
#include <cmath>

Tracker::Tracker(URCommander &commander)
  : commander_(commander)
{
  LOG_INFO("Initializing ur_driver/tracked_point subscriber");
  point_sub_ = nh_.subscribe("ur_driver/tracked_point", 1, &Tracker::point_cb, this);
  LOG_INFO("Initialized ur_driver/tracked_point");
}

void Tracker::point_cb(trajectory_msgs::JointTrajectoryPointConstPtr point) {
  switch (state_)
  {
    case RobotState::Running: {
      last_track_ = ros::Time::now();

      // Send speedj command
      std::array<double, 6> cmd{{point->velocities[0],
                                    point->velocities[1],
                                    point->velocities[2],
                                    point->velocities[3],
                                    point->velocities[4],
                                    point->velocities[5]}};

      commander_.speedj(cmd, 10);
      break;
    }
    case RobotState::EmergencyStopped:
      LOG_ERROR("Not acting on point callback. Robot is emergency stopped");
      break;
    case RobotState::ProtectiveStopped:
      LOG_ERROR("Not acting on point callback. Robot is protective stopped");
      break;
    case RobotState::Error:
      LOG_ERROR("Not acting on point callback. Robot is not ready, check robot_mode");
      break;
    default:
      LOG_ERROR("Not acting on point callback. Robot is in undefined state");
      break;
  }
}

void Tracker::onRobotStateChange(RobotState state)
{
  state_ = state;
}

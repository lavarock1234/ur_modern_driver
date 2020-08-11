/*
 * Copyright 2020, Xuchu (Dennis) Ding dennis@bluehill.coffee
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

#include "ur_modern_driver/ros/rtde_trajectory_follower.h"
#include <ros/ros.h>
#include <ur_msgs/TrajectoryFeedback.h>
#include <cmath>
#include <fstream>

RTDETrajectoryFollower::RTDETrajectoryFollower(std::string &robot_ip)
  : running_(false)
  , servoj_time_(0.008)
  , control_interface_(robot_ip, 30004, true)
{
  //ros::param::get("~servoj_time", servoj_time_);
  dt_ = servoj_time_ / 8.0;

  status_pub_ = nh_.advertise<ur_msgs::TrajectoryFeedback>("ur_driver/tracking_status", 20);
}

bool RTDETrajectoryFollower::start(double servoj_gain, double servoj_lookahead_time)
{
  if (running_)
    return true;  // not sure

  control_interface_.reuploadScript();

  current_servoj_gain_ = servoj_gain;
  current_servoj_lookahead_time_ = servoj_lookahead_time;
  LOG_INFO("Running RTDE controller interface with servoj gain: %f and lookahead_time: %f", current_servoj_gain_, current_servoj_lookahead_time_);
  return (running_ = true);
}

double RTDETrajectoryFollower::interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel)
{
  if(t < 0) {
      ROS_INFO_THROTTLE(1.0, "Detected interpolation loop t (%f) < 0", t);
      return p0_pos;
  }
  if(t > T) {
      ROS_INFO_THROTTLE(1.0, "Detected interpolation loop t (%f) > T (%f)", t, T);
      return p1_pos;
  }
  using std::pow;
  double a = p0_pos;
  double b = p0_vel;
  double c = (-3 * a + 3 * p1_pos - 2 * T * b - T * p1_vel) / pow(T, 2);
  double d = (2 * a - 2 * p1_pos + T * b + T * p1_vel) / pow(T, 3);
  return a + b * t + c * pow(t, 2) + d * pow(t, 3);
}

bool RTDETrajectoryFollower::execute(const JointAngle &positions) {
  return control_interface_.servoJ(positions, 0, 0, dt_, current_servoj_lookahead_time_, current_servoj_gain_);
}

bool RTDETrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt, std::atomic<bool> &paused)
{
  if (!running_)
    return false;

  using namespace std::chrono;
  typedef duration<double> double_seconds;
  typedef high_resolution_clock Clock;
  typedef Clock::time_point Time;

  auto &last = trajectory[trajectory.size() - 1];
  auto &prev = trajectory[0];

  Time t0 = Clock::now();
  Time latest = t0;

  JointAngle positions(6);

  uint32_t idx = 0;
  for (auto const &point : trajectory)
  {
    // skip t0
    if (&point == &prev)
      continue;

    if (interrupt)
      break;

    while (paused) {
      if (interrupt) {
        break;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }

    auto duration = point.time_from_start - prev.time_from_start;
    double d_s = duration_cast<double_seconds>(duration).count();

    // interpolation loop
    double t = 0;
    while (!interrupt)
    {
      while (paused) {
        if (interrupt) {
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
      }
      if (interrupt)
        break;      

      for (size_t j = 0; j < positions.size(); j++) {
          positions[j] =
            interpolate(t, d_s, prev.positions[j], point.positions[j], prev.velocities[j], point.velocities[j]);
      }

      auto t_start = Clock::now();
      if (!execute(positions))
        return false;
      auto t_stop = Clock::now();
      auto t_duration = std::chrono::duration<double>(t_stop - t_start);
      if (t_duration.count() < dt_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_ - t_duration.count()));
      }
      t += duration_cast<double_seconds>(Clock::now() - t_start).count();

      if(t > d_s) {
          break;
      }
    }

    prev = point;

    // Update current tracking status
    ur_msgs::TrajectoryFeedback status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.current_idx = idx;
    status_msg.goal_id = current_gh_id;
    status_pub_.publish(status_msg);
    idx++;
  }

  if (interrupt) {    
    return true;
  }

  // In theory it's possible the last position won't be sent by
  // the interpolation loop above but rather some position between
  // t[N-1] and t[N] where N is the number of trajectory points.
  // To make sure this does not happen the last position is sent
  return execute(from_array(last.positions));
}

void RTDETrajectoryFollower::stop()
{

  //control_interface_.stopScript();

  if (!running_)
    return;


  running_ = false;
}

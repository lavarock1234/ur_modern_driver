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

  // Upload script only if script is dirty
  if (script_handler_) {
    if (!script_handler_->isControlScriptActive()) {
      control_interface_.reuploadScript();
      script_handler_->setControlScriptActive(true);
    }
  } else {
    LOG_ERROR("Script handler not set!");
  }

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

bool RTDETrajectoryFollower::execute_moveJ(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt, std::atomic<bool> &paused) {
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
  std::chrono::microseconds t(0);
  auto t_begin = Clock::now();
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

    // Calculate an estimate of speed
    double speed = -1;
    double d_s = duration_cast<double_seconds>(point.time_from_start - prev.time_from_start).count();
    // compute lead axis speed rad/s
    for (unsigned i = 0; i < point.positions.size(); i++)
    {
      if(std::abs(point.positions[i] - prev.positions[i]) / d_s > speed) {
        speed = std::abs(point.positions[i] - prev.positions[i]) / d_s;
      }
    }
    LOG_INFO("SENDING [moveJ] cmd with speed: %f [rad/s]", speed);
    control_interface_.moveJ(from_array(point.positions), speed);
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

  // Show timing
  auto desired = duration_cast<double_seconds>(last.time_from_start).count();
  auto total= duration_cast<double_seconds>(Clock::now() - t_begin).count();
  LOG_INFO("Action timing [moveJ]: desired: %f [s], completed: %f [s], delayed: %f%%",  desired, total, 100.0 * (total - desired) / desired);
  return true;
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
  std::chrono::microseconds t(0);
  auto t_begin = Clock::now();
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

    double d_s = duration_cast<double_seconds>(point.time_from_start - prev.time_from_start).count();    
    while (t <= point.time_from_start) {
      while (paused) {
        if (interrupt) {
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
      }
      if (interrupt)
        break;

      auto t_start = Clock::now();
      double d_t = duration_cast<double_seconds>(t - prev.time_from_start).count();
      double d_t_s = (std::sin(d_t/d_s*M_PI/3 - M_PI/6)+0.5) * d_s;
      for (size_t j = 0; j < positions.size(); j++) {
          positions[j] =
            interpolate(d_t_s, d_s, prev.positions[j], point.positions[j], prev.velocities[j], point.velocities[j]);
      }
      if (!execute(positions))
        return false;
      auto t_stop = Clock::now();
      auto t_duration = std::chrono::duration<double>(t_stop - t_start);
      if (t_duration.count() < dt_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_ - t_duration.count()));
      }
      t += duration_cast<microseconds>(Clock::now() - t_start);
      //std::cout << "tfs = " << point.time_from_start.count() * 1e-6 << " dt = " << d_t << " Current t = " << t.count() * 1e-6 << "\n";
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

  // Show timing
  auto desired = duration_cast<double_seconds>(last.time_from_start).count();
  auto total= duration_cast<double_seconds>(Clock::now() - t_begin).count();
  LOG_INFO("Action timing: desired: %f [s], completed: %f [s], delayed: %f%%",  desired, total, 100.0 * (total - desired) / desired);

  // In theory it's possible the last position won't be sent by
  // the interpolation loop above but rather some position between
  // t[N-1] and t[N] where N is the number of trajectory points.
  // To make sure this does not happen the last position is sent
  if(!execute(from_array(last.positions))) {
    return false;
  }
  return true;
}

bool RTDETrajectoryFollower::servo_stop() {
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  return control_interface_.servoStop();
}


void RTDETrajectoryFollower::stop()
{

  //control_interface_.stopScript();

  if (!running_)
    return;

  running_ = false;
}

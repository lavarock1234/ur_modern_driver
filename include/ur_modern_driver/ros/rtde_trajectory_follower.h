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

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <ur_rtde/rtde_control_interface.h>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/action_trajectory_follower_interface.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"
#include "ur_modern_driver/ros/urscript_handler.h"

class RTDETrajectoryFollower : public ActionTrajectoryFollowerInterface
{
private:
  ros::NodeHandle nh_;
  ros::Publisher status_pub_;
  std::atomic<bool> running_;
  std::array<double, 6> last_positions_;
  double servoj_time_;
  double dt_;
  double interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel);

  // These values will be updated in start
  double current_servoj_gain_ = 100;
  double current_servoj_lookahead_time_ = 0.2;

  // Control interface
  ur_rtde::RTDEControlInterface control_interface_;

  // Script handler
  URScriptHandler* script_handler_;

public:
  RTDETrajectoryFollower(std::string &robot_ip);
  typedef std::vector<double> JointAngle;

  bool execute(const JointAngle &positions);
  bool start(double servoj_gain, double servoj_lookahead_time) override;
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt, std::atomic<bool> &paused) override;
  bool execute_moveJ(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt, std::atomic<bool> &paused) override;
  void stop() override;
  bool servo_stop() override;

  inline ur_rtde::RTDEControlInterface* get_control_interface() {
    return &control_interface_;
  }

  inline void set_script_handler(URScriptHandler* script_handler) {
    script_handler_ = script_handler;
  }

  virtual ~RTDETrajectoryFollower(){
    control_interface_.stopScript();
  };

  inline JointAngle from_array(std::array<double, 6> positions) {
    std::vector<double> ret(positions.begin(), positions.end());
    return ret;
  }
};

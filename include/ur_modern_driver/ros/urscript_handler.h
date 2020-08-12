/*
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

#include <ros/ros.h>
#include <string>

#include <ur_rtde/rtde_control_interface.h>

#include "std_msgs/String.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"

class URScriptHandler : public Service
{
private:
  ros::NodeHandle nh_;
  URCommander &commander_;
  ros::Subscriber urscript_sub_;
  RobotState state_;
  ur_rtde::RTDEControlInterface* control_interface_ = nullptr;
  std::atomic<bool> control_script_active_; ///< True if control script is active; False otherwise
public:
  URScriptHandler(URCommander &commander);
  void urscriptInterface(const std_msgs::String::ConstPtr &msg);
  void onRobotStateChange(RobotState state);
  void enableRTDEMode(ur_rtde::RTDEControlInterface* control_interface) {
    control_interface_ = control_interface;
  }
  bool isControlScriptActive() const {
    return control_script_active_;
  }
  void setControlScriptActive(bool control_script_active) {
    control_script_active_ = control_script_active;
  }
};

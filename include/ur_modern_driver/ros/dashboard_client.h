/*
 * Copyright 2020 Xuchu (Dennis) Ding
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

#include <ur_rtde/dashboard_client.h>

#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_dashboard_msgs/GetRobotMode.h"

class DashboardClient : public Service
{
private:
  ros::NodeHandle nh_;
  std::shared_ptr<ur_rtde::DashboardClient> db_client_;

  ros::ServiceServer mode_service_, close_safety_popup_service_, power_on_service_, brake_release_service_, 
    power_off_service_, unlock_protective_stop_service_;

  RobotState state_;
public:
  DashboardClient(std::string host_ip) {
    // Create a connection to the dashboard server
    db_client_ = std::make_shared<ur_rtde::DashboardClient>(host_ip);
    db_client_->connect();

    mode_service_ = nh_.advertiseService("/ur_driver/dashboard/get_robot_mode", &DashboardClient::mode_svc, this);
    close_safety_popup_service_ = nh_.advertiseService("/ur_driver/dashboard/close_safety_popup", &DashboardClient::close_safety_popup_svc, this);
    power_on_service_ = nh_.advertiseService("/ur_driver/dashboard/power_on", &DashboardClient::power_on_svc, this);
    brake_release_service_ = nh_.advertiseService("/ur_driver/dashboard/brake_release", &DashboardClient::brake_release_svc, this);
    power_off_service_ = nh_.advertiseService("/ur_driver/dashboard/power_off", &DashboardClient::power_off_svc, this);
    unlock_protective_stop_service_ = nh_.advertiseService("/ur_driver/dashboard/unlock_protective_stop", &DashboardClient::unlock_protective_stop_svc, this);
  }
  void onRobotStateChange(RobotState state);

  bool mode_svc(ur_dashboard_msgs::GetRobotModeRequest& req, ur_dashboard_msgs::GetRobotModeResponse& resp) {
    auto robot_mode_str = db_client_->robotmode();
    switch (ur_rtde::parseRobotMode(robot_mode_str)) {
      case ur_rtde::RobotMode::NO_CONTROLLER:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::NO_CONTROLLER;
        break;
      case ur_rtde::RobotMode::DISCONNECTED:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::DISCONNECTED;
        break;
      case ur_rtde::RobotMode::CONFIRM_SAFETY:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::CONFIRM_SAFETY;
        break;
      case ur_rtde::RobotMode::BOOTING:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::BOOTING;
        break;
      case ur_rtde::RobotMode::POWER_OFF:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::POWER_OFF;
        break;
      case ur_rtde::RobotMode::POWER_ON:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::POWER_ON;
        break;
      case ur_rtde::RobotMode::IDLE:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::IDLE;
        break;
      case ur_rtde::RobotMode::BACKDRIVE:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::BACKDRIVE;
        break;
      case ur_rtde::RobotMode::RUNNING:
        resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::RUNNING;
        break;
    }
    resp.answer = robot_mode_str;
    resp.success = true;
    return true;
  }

  bool close_safety_popup_svc(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    db_client_->closePopup();
    resp.success = true;
    return true;
  }

  bool power_on_svc(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    db_client_->powerOn();
    resp.success = true;
    return true;
  }

  bool brake_release_svc(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    db_client_->brakeRelease();
    resp.success = true;
    return true;
  }

  bool power_off_svc(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    db_client_->powerOff();
    resp.success = true;
    return true;
  }

  bool unlock_protective_stop_svc(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    db_client_->unlockProtectiveStop();
    resp.success = true;
    return true;
  }  
};

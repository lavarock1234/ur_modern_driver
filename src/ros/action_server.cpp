/*
 * Copyright 2018 Jarek Potiuk (low bandwidth trajectory follower)
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

#include <cmath>
#include "ur_modern_driver/ros/action_server.h"

ActionServer::ActionServer(ActionTrajectoryFollowerInterface& follower, std::vector<std::string>& joint_names,
                           double max_velocity)
  : as_(nh_, "follow_joint_trajectory", boost::bind(&ActionServer::onGoal, this, _1),
        boost::bind(&ActionServer::onCancel, this, _1), false)
  , joint_names_(joint_names)
  , joint_set_(joint_names.begin(), joint_names.end())
  , max_velocity_(max_velocity)
  , interrupt_traj_(false)
  , pause_traj_(false)
  , has_goal_(false)
  , running_(false)
  , follower_(follower)
  , state_(RobotState::Error)
{
  pause_service_ = nh_.advertiseService("/ur_driver/set_program_state", &ActionServer::onPause, this);
}

void ActionServer::start()
{
  if (running_)
    return;

  LOG_INFO("Starting ActionServer");
  running_ = true;
  tj_thread_ = thread(&ActionServer::trajectoryThread, this);
  as_.start();
}

void ActionServer::onRobotStateChange(RobotState state)
{
  state_ = state;

  // don't interrupt if everything is fine
  if (state == RobotState::Running)
    return;

  // don't retry interrupts
  if (interrupt_traj_ || !has_goal_)
    return;

  // on successful lock we're not executing a goal so don't interrupt
  if (tj_mutex_.try_lock())
  {
    tj_mutex_.unlock();
    return;
  }

  interrupt_traj_ = true;
  // wait for goal to be interrupted and automagically unlock when going out of scope
  std::lock_guard<std::mutex> lock(tj_mutex_);

  Result res;
  res.error_code = -100;
  res.error_string = "Robot safety stop";
  curr_gh_.setAborted(res, res.error_string);
}

bool ActionServer::updateState(RTShared& data)
{
  q_actual_ = data.q_actual;
  qd_actual_ = data.qd_actual;
  return true;
}

bool ActionServer::consume(RTState_V1_6__7& state)
{
  return updateState(state);
}
bool ActionServer::consume(RTState_V1_8& state)
{
  return updateState(state);
}
bool ActionServer::consume(RTState_V3_0__1& state)
{
  return updateState(state);
}
bool ActionServer::consume(RTState_V3_2__3& state)
{
  return updateState(state);
}

void ActionServer::onGoal(GoalHandle gh)
{
  Result res;
  res.error_code = -100;

  LOG_INFO("Received new goal");

  if (!validate(gh, res) || !try_execute(gh, res))
  {
    LOG_WARN("Goal error: %s", res.error_string.c_str());
    gh.setRejected(res, res.error_string);
  }
}

void ActionServer::onCancel(GoalHandle gh)
{
  interrupt_traj_ = true;
  // wait for goal to be interrupted
  std::lock_guard<std::mutex> lock(tj_mutex_);

  Result res;
  res.error_code = -100;
  res.error_string = "Goal cancelled by client";
  gh.setCanceled(res);
}

bool ActionServer::validate(GoalHandle& gh, Result& res)
{
  return validateState(gh, res) && validateJoints(gh, res) && validateTrajectory(gh, res);
}

bool ActionServer::validateState(GoalHandle& gh, Result& res)
{
  switch (state_)
  {
    case RobotState::EmergencyStopped:
      res.error_string = "Robot is emergency stopped";
      return false;

    case RobotState::ProtectiveStopped:
      res.error_string = "Robot is protective stopped";
      return false;

    case RobotState::Error:
      res.error_string = "Robot is not ready, check robot_mode";
      return false;

    case RobotState::Running:
      return true;

    default:
      res.error_string = "Undefined state";
      return false;
  }
}

bool ActionServer::validateJoints(GoalHandle& gh, Result& res) {
  auto goal = gh.getGoal();
  auto const& joints = goal->trajectory.joint_names;
  std::set<std::string> goal_joints(joints.begin(), joints.end());

  if (goal_joints == joint_set_)
    return true;

  res.error_code = Result::INVALID_JOINTS;
  res.error_string = "Invalid joint names for goal\n";
  res.error_string += "Expected: ";
  std::for_each(goal_joints.begin(), goal_joints.end(),
                [&res](std::string joint) { res.error_string += joint + ", "; });
  res.error_string += "\nFound: ";
  std::for_each(joint_set_.begin(), joint_set_.end(), [&res](std::string joint) { res.error_string += joint + ", "; });
  return false;
}

bool ActionServer::validateTrajectory(GoalHandle& gh, Result& res)
{
  auto goal = gh.getGoal();
  res.error_code = Result::INVALID_GOAL;

  // must at least have one point
  if (goal->trajectory.points.size() < 1)
    return false;

  for (auto const& point : goal->trajectory.points)
  {
    if (point.velocities.size() != joint_names_.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of velocities";
      return false;
    }

    if (point.positions.size() != joint_names_.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of positions";
      return false;
    }

    for (auto const& velocity : point.velocities)
    {
      if (!std::isfinite(velocity))
      {
        res.error_string = "Received a goal with infinities or NaNs in velocity";
        return false;
      }
      if (std::fabs(velocity) > max_velocity_)
      {
        res.error_string =
            "Received a goal with velocities that are higher than max_velocity_ " + std::to_string(max_velocity_);
        return false;
      }
    }
    for (auto const& position : point.positions)
    {
      if (!std::isfinite(position))
      {
        res.error_string = "Received a goal with infinities or NaNs in positions";
        return false;
      }
    }
  }

  // todo validate start position?

  return true;
}

inline std::chrono::microseconds convert(const ros::Duration& dur)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds(dur.sec) +
                                                               std::chrono::nanoseconds(dur.nsec));
}

bool ActionServer::try_execute(GoalHandle& gh, Result& res)
{
  if (!running_)
  {
    res.error_string = "Internal error";
    return false;
  }
  if (!tj_mutex_.try_lock())
  {
    interrupt_traj_ = true;
    res.error_string = "Received another trajectory";
    curr_gh_.setAborted(res, res.error_string);
    tj_mutex_.lock();
    // todo: make configurable
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  // locked here
  curr_gh_ = gh;
  interrupt_traj_ = false;
  pause_traj_ = false;
  has_goal_ = true;
  tj_mutex_.unlock();
  tj_cv_.notify_one();
  return true;
}

std::vector<size_t> ActionServer::reorderMap(std::vector<std::string> goal_joints) {
  std::vector<size_t> indecies;
  for (auto const& aj : joint_names_) {
    size_t j = 0;
    for (auto const& gj : goal_joints) {
      if (aj == gj)
        break;
      j++;
    }
    indecies.push_back(j);
  }
  return indecies;
}

void ActionServer::trajectoryThread() {
  LOG_INFO("Trajectory thread started");

  while (running_) {
    std::unique_lock<std::mutex> lk(tj_mutex_);
    if (!tj_cv_.wait_for(lk, std::chrono::milliseconds(100), [&] { return running_ && has_goal_; }))
      continue;

    LOG_INFO("Trajectory received and accepted");
    curr_gh_.setAccepted();

    auto goal = curr_gh_.getGoal();
    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(goal->trajectory.points.size() + 1);

    // joint names of the goal might have a different ordering compared
    // to what URScript expects so need to map between the two
    auto mapping = reorderMap(goal->trajectory.joint_names);

    LOG_INFO("Translating trajectory");

    auto const& fp = goal->trajectory.points[0];
    auto fpt = convert(fp.time_from_start);

    // make sure we have a proper t0 position
    if (fpt > std::chrono::microseconds(0)) {
      LOG_INFO("Trajectory without t0 recieved, inserting t0 at current position");
      trajectory.push_back(TrajectoryPoint(q_actual_, qd_actual_, std::chrono::microseconds(0)));
    }

    for (auto const& point : goal->trajectory.points) {
      std::array<double, 6> pos, vel;
      for (size_t i = 0; i < 6; i++) {
        size_t idx = mapping[i];
        pos[idx] = point.positions[i];
        vel[idx] = point.velocities[i];
      }
      auto t = convert(point.time_from_start);
      trajectory.push_back(TrajectoryPoint(pos, vel, t));
    }

    double t = std::chrono::duration_cast<std::chrono::duration<double>>(trajectory[trajectory.size() - 1].time_from_start).count();
    LOG_INFO("Executing trajectory with %zu points and duration of %4.3fs", trajectory.size(), t);

    Result res;

    // Modified frame_id parameter pass through
    std::stringstream ss(goal->trajectory.header.frame_id);
    uint64_t goal_id; double sharpness; double servoj_gain; double servoj_lookahead_time;
    ss >> goal_id >> sharpness;
    bool servo_stop = false;
    if (sharpness < 0)
    {
      servo_stop = true;
      sharpness = 0;
    }
    // convex combination between (100, 0.2) -> (500, 0.05)
    // FIXME configure as rosparam
    servoj_gain = (1 - sharpness) * 100 + sharpness * 500;
    servoj_lookahead_time = (1 - sharpness) * 0.2 + sharpness * 0.05;
    LOG_INFO("Received trajectory parameters goal_id: %d, gain: %f, lookahead_time: %f", (int)goal_id, servoj_gain, servoj_lookahead_time);
    LOG_INFO("Attempting to start follower %p, servo_stop? %d", &follower_, int(servo_stop));
    if (follower_.start(servoj_gain, servoj_lookahead_time)) {
      follower_.current_gh_id = std::to_string(goal_id);
      if (servo_stop ?
          follower_.servo_stop() :
          follower_.execute(trajectory, interrupt_traj_, pause_traj_)
          ) {
        // interrupted goals must be handled by interrupt trigger
        if (!interrupt_traj_) {
          LOG_INFO("Trajectory executed successfully");
          res.error_code = Result::SUCCESSFUL;
          curr_gh_.setSucceeded(res);
        } else {
          LOG_INFO("Trajectory interrupted");
        }
      } else {
        LOG_INFO("Trajectory failed");
        res.error_code = -100;
        res.error_string = "Connection to robot was lost";
        curr_gh_.setAborted(res, res.error_string);
      }
      LOG_INFO("Attempting to stop follower %p", &follower_);
      follower_.stop();
      LOG_INFO("Follower stopped");
    } else {
      LOG_ERROR("Failed to start trajectory follower!");
      res.error_code = -100;
      res.error_string = "Robot connection could not be established";
      curr_gh_.setAborted(res, res.error_string);
    }
    has_goal_ = false;
    lk.unlock();
  }
}

bool ActionServer::onPause(ur_msgs::ProgramStateRequest &req, ur_msgs::ProgramStateResponse &resp) {
  switch (req.state) {
    case ur_msgs::ProgramStateRequest::PAUSE:
      ROS_INFO("Setting pause state %d -> %d", (bool) pause_traj_, true);
      pause_traj_ = true;
      break;
    case ur_msgs::ProgramStateRequest::RESUME:
      ROS_INFO("Setting pause state %d -> %d", (bool) pause_traj_, false);
      pause_traj_ = false;
      break;
    default:
      ROS_ERROR("Unexpected request state (%d)", int(req.state));
  }
  resp.success = true;
  return true;
}

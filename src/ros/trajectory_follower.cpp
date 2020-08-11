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
#include "ur_modern_driver/ros/trajectory_follower.h"
#include <endian.h>
#include <ros/ros.h>
#include <ur_msgs/TrajectoryFeedback.h>
#include <cmath>
#include <fstream>

static const int32_t MULT_JOINTSTATE_ = 1000000;
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def driverProg():
	MULT_jointstate = {{JOINT_STATE_REPLACE}}

	SERVO_IDLE = 0
	SERVO_RUNNING = 1
	cmd_servo_state = SERVO_IDLE
	cmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  global MAX_JOINT_DIFFERENCE = 0.02
  global JOINT_NUM = 6

	def set_servo_setpoint(q):
		enter_critical
		cmd_servo_state = SERVO_RUNNING
		cmd_servo_q = q
		exit_critical
	end

  def close_to_current(position):
    local l_current_position = get_actual_joint_positions()
    local l_index = 0
    while l_index < JOINT_NUM:
      if norm(position[l_index] - l_current_position[l_index]) > MAX_JOINT_DIFFERENCE:
          return False
      end
      l_index = l_index + 1
    end
    return True
  end

  def get_max_joint_difference(position):
    local l_current_position = get_actual_joint_positions()
    local l_index = 0
    local l_max_joint_difference = 0.0
    while l_index < JOINT_NUM:
      if norm(position[l_index] - l_current_position[l_index]) > l_max_joint_difference:
        l_max_joint_difference = norm(position[l_index] - l_current_position[l_index])
      end
      l_index = l_index + 1
    end
    return l_max_joint_difference
  end

	thread servoThread():
		state = SERVO_IDLE
		while True:
			enter_critical
			q = cmd_servo_q
			do_brake = False
			if (state == SERVO_RUNNING) and (cmd_servo_state == SERVO_IDLE):
				do_brake = True
			end
			state = cmd_servo_state
			cmd_servo_state = SERVO_IDLE
			exit_critical
			if do_brake:
				stopj(1.0)
				sync()
			elif state == SERVO_RUNNING:
				servoj(q, {{SERVO_J_REPLACE}})
			else:
				sync()
			end
		end
	end

  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  textmsg("Starting bluehill program")
  thread_servo = run servoThread()
  keepalive = 1
  max_joint_difference = 0.0
  while keepalive > 0:
	  params_mult = socket_read_binary_integer(6+1)
	  if params_mult[0] > 0:
		  q = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
		  keepalive = params_mult[7]
      if keepalive == 0:
        break
      end
		  set_servo_setpoint(q)
	  end
  end
  textmsg("Found max joint difference: ", max_joint_difference)
  sleep(.1)
  socket_close()
  kill thread_servo
end
)";

TrajectoryFollower::TrajectoryFollower(URCommander &commander, std::string &reverse_ip, int reverse_port,
                                       bool version_3)
  : running_(false)
  , commander_(commander)
  , server_(reverse_port)
  , servoj_time_(0.008)
  , log_servoj_(false)
{
  ros::param::get("~servoj_time", servoj_time_);

  status_pub_ = nh_.advertise<ur_msgs::TrajectoryFeedback>("ur_driver/tracking_status", 20);

  std::string res(POSITION_PROGRAM);
  res.replace(res.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));

  if (!version_3) {
    LOG_ERROR("Failed to run trajectory, version is below 3");
    std::exit(-1);
  }

  res.replace(res.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), reverse_ip);
  res.replace(res.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  program_ = res;

  if (!server_.bind())
  {
    LOG_ERROR("Failed to bind server, the port %d is likely already in use", reverse_port);
    std::exit(-1);
  }
}

bool TrajectoryFollower::start(double servoj_gain, double servoj_lookahead_time)
{
  if (running_)
    return true;  // not sure

  LOG_INFO("Uploading trajectory program to robot with servoj gain: %f and lookahead_time: %f", servoj_gain, servoj_lookahead_time);

  // Updating program
  std::string updated_program = program_;
  std::ostringstream out;
  out << "t=" << std::fixed << std::setprecision(4) << servoj_time_ << ", lookahead_time=" << servoj_lookahead_time << ", gain=" << servoj_gain;
  updated_program.replace(updated_program.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());

  // Clear servoj log
  servoj_log_.clear();

  // Upload program
  if (!commander_.uploadProg(updated_program))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }

  LOG_DEBUG("Awaiting incoming robot connection");

  if (!server_.accept())
  {
    LOG_ERROR("Failed to accept incoming robot connection");
    return false;
  }

  LOG_DEBUG("Robot successfully connected");
  return (running_ = true);
}

bool TrajectoryFollower::execute(const std::array<double, 6> &positions, bool keep_alive)
{
  if (!running_)
    return false;

  //  LOG_INFO("servoj([%f,%f,%f,%f,%f,%f])", positions[0], positions[1], positions[2], positions[3], positions[4],
  //  positions[5]);
  if(log_servoj_)    
    servoj_log_.push_back(positions);

  last_positions_ = positions;

  uint8_t buf[sizeof(uint32_t) * 7];
  uint8_t *idx = buf;

  for (auto const &pos : positions)
  {
    int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE_);
    val = htobe32(val);
    idx += append(idx, val);
  }

  int32_t val = htobe32(static_cast<int32_t>(keep_alive));
  append(idx, val);

  size_t written;
  return server_.write(buf, sizeof(buf), written);
}

double TrajectoryFollower::interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel)
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

bool TrajectoryFollower::execute(std::array<double, 6> &positions)
{
  return execute(positions, true);
}

bool TrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt, std::atomic<bool> &paused)
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

  std::array<double, 6> positions;

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

      if (!execute(positions, true))
        return false;

      Time servoj_time = Clock::now();
      //std::this_thread::sleep_for(std::chrono::milliseconds((int)((servoj_time_ * 1000) / 4.)));
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      //t += 0.000500;
      t += duration_cast<double_seconds>(Clock::now() - servoj_time).count();

      if(t > d_s) {
          if (!execute(point.positions, true)) {
              return false;
          } else {
              break;
          }
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

  // Setting keep_alive false breaks URScript loop, comment to revert to previous behavior
  execute(last_positions_, false);

  if (interrupt) {    
    return true;
  }

  // In theory it's possible the last position won't be sent by
  // the interpolation loop above but rather some position between
  // t[N-1] and t[N] where N is the number of trajectory points.
  // To make sure this does not happen the last position is sent
  //return execute(last.positions, true);
  return true;
}

void TrajectoryFollower::stop()
{
  if(log_servoj_) {
    std::string filename = std::string(getenv("HOME")) + "/" + std::to_string(ros::Time::now().toNSec()) + ".servoj";
    std::ofstream file(filename);
    if(!file.is_open()) {
      ROS_ERROR("Unable to open file %s for writing!", filename.c_str());
    } else {
      ROS_INFO("Writing servoj commands to: %s", filename.c_str());
      for(auto&& positions : servoj_log_) {
        file << positions[0] << " " << positions[1] << " " << positions[2] << " " << positions[3] << " " << positions[4] << " " << positions[5] << "\n";
      }
    }
    file.close();
  }

  if (!running_)
    return;

  // std::array<double, 6> empty;
  // execute(empty, false);

  server_.disconnectClient();

  running_ = false;
}

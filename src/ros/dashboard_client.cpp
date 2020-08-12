#include "ur_modern_driver/ros/dashboard_client.h"

void DashboardClient::onRobotStateChange(RobotState state)
{
  state_ = state;
}

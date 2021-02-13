#include <move_rt/emergencystop.hpp>

EmergencyStop::EmergencyStop(ros::NodeHandle &n,
                             Eigen::Matrix<long double, Eigen::Dynamic, 1> &_q)
    : PriorityLevel(n), q(_q) {

  ROS_DEBUG_STREAM("EmergencyStop::EmergencyStop: enter contructor ");

  PriorityLevel::update_task();

  JK.setIdentity();

  A.setIdentity();
  enabled = true;

  position_initialized = false;

  ROS_DEBUG_STREAM("EmergencyStop::EmergencyStop: contructor finished");
}

EmergencyStop::~EmergencyStop() {}

void EmergencyStop::updateConstraints() {
  for (int i = 0; i < m; ++i)
    x(i, 0) = q(i, 0);

  if (!position_initialized) {
    for (int i = 0; i < m; ++i) {
      xd(i, 0) = q(i, 0);
    }
    if (initialized)
      position_initialized = true;
  }

  if (!enabled)
    for (int i = 0; i < m; ++i)
      xd(i, 0) = q(i, 0);
}

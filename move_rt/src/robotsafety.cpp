#include <move_rt/robotsafety.hpp>

RobotSafety::RobotSafety(ros::NodeHandle &nodehandle,
                         Eigen::Matrix<long double, Eigen::Dynamic, 1> &d_q)
    : PriorityLevel(nodehandle), joint_vel(d_q) {
  ROS_DEBUG_STREAM("enter RobotSafeStop contructor ");

  PriorityLevel::update_task();

  sub_safetyactivation = nodehandle.subscribe<std_msgs::Bool>(
      "safety_activation", 1, &RobotSafety::callbackactivation, this);

  JK.setIdentity();

  ROS_DEBUG_STREAM("exit RobotSafeStop contructor ");

  safety_val = false;
}

RobotSafety::~RobotSafety() {}

void RobotSafety::updateA() {
  A.setZero();
  if (safety_val)
    A.setIdentity();
}

void RobotSafety::updateConstraints() {
  ROS_DEBUG_STREAM("size of xd:  " << x.size());
  for (int i = 0; i < m; i++) {
    xd(i, 0) = 0;
    x(i, 0) = joint_vel(i, 0);
  }
  ROS_DEBUG_STREAM("x \n" << x);
  ROS_DEBUG_STREAM("xd \n" << xd);
}

string RobotSafety::get_task() { return "RobotSafetyStop"; }

// void RobotSafety::show()
//{
//    PriorityLevel::show();

//    std::cout<<"qmax= "<<qmax<<"\n";
//    std::cout<<"qmin= "<<qmin<<"\n";

//}
void RobotSafety::callbackactivation(const std_msgs::Bool::ConstPtr &msg) {
  safety_val = msg->data;
}

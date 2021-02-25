#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_rt/ExecutingTrajectoryAction.h>
#include <move_rt/TaskParamUpdate.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_traj_action_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<move_rt::ExecutingTrajectoryAction> ac(
      "simple_joint_trajectory");

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  ros::NodeHandle n;
  ros::ServiceClient disable_joint_control_client =
      n.serviceClient<move_rt::TaskParamUpdate>("JointPosition/setEnable");
  ros::ServiceClient disable_ee_control_client =
      n.serviceClient<move_rt::TaskParamUpdate>("EePosition/setEnable");
  ros::ServiceClient disable_emergency_stop_client =
      n.serviceClient<move_rt::TaskParamUpdate>("EmergencyStop/setEnable");

  move_rt::TaskParamUpdate srv;
    
  srv.request.data.push_back(1);

  if (disable_joint_control_client.call(srv)) {
    ROS_INFO("joint_traj_action_client: joint Position control enabled");
  } else {
    ROS_INFO(
        "joint_traj_action_client: error while enabling joint Position control");
    return -1;
  }
  
  srv.request.data.clear();  
  srv.request.data.push_back(0);

  if (disable_ee_control_client.call(srv)) {
    ROS_INFO("joint_traj_action_client: Workspace Position control disabled");
  } else {
    ROS_INFO(
        "joint_traj_action_client: error while disabling Workspace Position control");
    return -1;
  }

  if (disable_emergency_stop_client.call(srv)) {
    ROS_INFO("joint_traj_action_client: Emergency Stop released");
  } else {
    ROS_INFO("joint_traj_action_client: error while releasing Emergency Stop");
    return -1;
  }

  // Define the goal
  // string trajectory_name
  // float64 ee_error_th  # Specify error threshold
  move_rt::ExecutingTrajectoryGoal goal;
  goal.trajectory_name = "joint_traj_test";
  goal.ee_error_th = 0.1;

  ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout =
      ac.waitForResult(); // waitForResult(ros::Duration(30.0))

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else
    ROS_INFO("Action did not finish before the time out. State: %s",
             ac.getState().toString().c_str());

  // exit
  return 0;
}

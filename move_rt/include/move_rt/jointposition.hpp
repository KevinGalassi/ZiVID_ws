#ifndef JOINTPOSITION_H
#define JOINTPOSITION_H

#include <move_rt/prioritylevel.hpp>

static bool setsEqual(const std::vector<std::string> &a,
                      const std::vector<std::string> &b);

/// @brief JointPosition is the robot joint position controller to interface
/// with the Moveit! planners
class JointPosition : public PriorityLevel {
public:
  /**
   * Constructor of JointPosition.
   *
   * @param nodehandle is a pointer to a ros nodeHandle
   * @param _q is a pointer to the robot joint positions
   * @param joint_state is a pointer to the robot joint state message
   */
  JointPosition(ros::NodeHandle &nodehandle,
                Eigen::Matrix<long double, Eigen::Dynamic, 1> &_q,
                sensor_msgs::JointState &joint_state);

  /**
   * Destructor of the JointPosition.
   */
  ~JointPosition();

  /**
   * updateConstraints: update the x and xd vectors of EePosition task.
   */
  void updateConstraints();

  /**
   * get_task: return the name of the actual class.
   */
  string get_task() { return "JointPosition"; };

  /**
   * get_JK: returns the task Jacobian.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };

private:
  int traj_length;
    
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>
      JTAS;
  typedef JTAS::GoalHandle GoalHandle;

  JTAS action_server_;
    
  void watchdog(const ros::TimerEvent &e);

  void goalCB(GoalHandle gh);

  void cancelCB(GoalHandle gh);

  void controllerStateCB(
      const control_msgs::JointTrajectoryControllerStateConstPtr &msg);

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;
  
  int trajectory_index;

  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Timer watchdog_timer_;

  ros::Time trajectory_start;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> goal_constraints_;
  std::map<std::string, double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;
  
  /**
   * Joint Setpoint Action Server
   */
  move_rt::ExecutingTrajectoryFeedback feedback_;
  move_rt::ExecutingTrajectoryResult result_;
  typedef actionlib::ActionServer<move_rt::ExecutingTrajectoryAction> JSAS;
  typedef JSAS::GoalHandle JS_GoalHandle;

  void JS_goalCB(JS_GoalHandle gh);
  void JS_cancelCB(JS_GoalHandle gh);

  bool JS_has_active_goal_;
  JSAS JS_action_server_;
  JS_GoalHandle JS_active_goal_;

  sensor_msgs::JointState &joint_state_;

  bool position_initialized;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> &q, q_err;

  control_msgs::JointTrajectoryControllerStateConstPtr last_controller_state_;

  control_msgs::JointTrajectoryControllerState trajectory_controller_state;

  ros::Publisher pub_controller_state;
  
  void callbackJointSetpoint(const std_msgs::Float64MultiArray::ConstPtr &msg);
  ros::Subscriber sub_joint_setpoint;
};

#endif // JOINTPOSITION_H

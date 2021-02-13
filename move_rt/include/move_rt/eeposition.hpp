#ifndef EEPOSITION_H
#define EEPOSITION_H

#include <move_rt/prioritylevel.hpp>
#include <move_rt/UpdateFrame.h>

namespace rvt = rviz_visual_tools;

/// @brief EePosition is the end-effector position controller
class EePosition : public PriorityLevel {
public:
  /**
   * Constructor of EePosition.
   *
   * @param nodehandle is a pointer to a ros nodeHandle
   * @param _JK is a pointer to the end-effector Jacobian
   */
  EePosition(ros::NodeHandle &nodehandle,
             Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> &_JK);

  /**
   * Destructor of the EePosition.
   */
  ~EePosition();

  /**
   * control: implements the controller used to compute the velocity that should
   * be applied on the end effector (dx).
   */
  void control();

  /**
   * updateJk: update the Jacobian of EePosition task.
   */
  void updateJk();

  /**
   * updateConstraints: update the x and xd vectors of EePosition task.
   */
  void updateConstraints();

  /**
   * get_task: return the name of the actual class.
   */
  string get_task();

  /**
   * show: prints the parameters of the current task controller.
   */
  void show();

  /**
   * get_JK: returns the task Jacobian.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return EePosition::JK;
  };

private:
  int traj_length;

  move_rt::ExecutingTrajectoryFeedback feedback_;
  move_rt::ExecutingTrajectoryResult result_;
  typedef actionlib::ActionServer<move_rt::ExecutingTrajectoryAction> EEAS;
  typedef EEAS::GoalHandle GoalHandle;

  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  EEAS action_server_;
  bool has_active_goal_;
  GoalHandle active_goal_;

  /**
   * serviceClearTrajectory: Callback function to clear the trajectory
   * visualization.
   */
  bool serviceClearTrajectory(move_rt::TaskParamUpdate::Request &req,
                              move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_clear_trajectory;

  /**
   * serviceSetToolFrame: Callback function to set the tool frame
   */
  bool serviceSetToolFrame(move_rt::UpdateFrame::Request &req,
                           move_rt::UpdateFrame::Response &res);
  ros::ServiceServer service_set_tool_frame;

  string base_link, ee_link, tf_prefix = "", current_path = "";

  void callbackEeSetpoint(const std_msgs::Float64MultiArray::ConstPtr &msg);
  ros::Subscriber sub_ee_setpoint;

  void callbackWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  ros::Subscriber sub_wrench;

  void callbackEeSetTrajectory(const std_msgs::String::ConstPtr &msg);
  ros::Subscriber sub_ee_set_trajectory;

  ros::Publisher ee_error_pub;
  ros::Publisher ee_position_pub;
  ros::Publisher ee_trajectory_pub;

  std_msgs::Float64MultiArray
      errMsg; /*it's defined by the arrayed version of the matrix and the layout
                 of the matrix*/
  std_msgs::Float64MultiArray posMsg;
  std_msgs::Float64MultiArray trajMsg;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> v;

  std::queue<Eigen::Matrix<long double, 13, 1>> traj_points;
  Eigen::Matrix<long double, 13, 1> traj_point;

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> &JK;
  bool valid_JK;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener; //(tfBuffer);
  geometry_msgs::TransformStamped transform_above;

  Vector3f force;
  Vector3f torque;

  Vector3f Kp;
  Vector3f Ka;
  Vector3f disp;

  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position;

  std::string group;
  std::string model;
  std::vector<std::string> link_names;

  robot_model_loader::RobotModelLoader *robot_model_loader_ptr;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::JointModelGroup *joint_model_group;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::string PLANNING_SCENE_SERVICE;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> quat, obj, x_diff, w, w2;
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> E;

  moveit_visual_tools::MoveItVisualTools *visual_tools = nullptr;
  std::vector<geometry_msgs::Pose> all_goals;

  void clear_traj_points(void) {
    traj_points = std::queue<Eigen::Matrix<long double, 13, 1>>();
  };
};

#endif // EEPOSITION_H

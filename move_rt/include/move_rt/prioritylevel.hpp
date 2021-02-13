#ifndef PRIORITYLEVEL_H
#define PRIORITYLEVEL_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <math.h>
#include <memory>
#include <queue>
#include <sstream>
#include <stdio.h> // defines FILENAME_MAX
#include <stdlib.h>
#include <string>
#include <unistd.h> // for getcwd()
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include <boost/bind.hpp>

#include <ros/ros.h> //ALWAYS need to include this

#include <move_rt/ExecutingTrajectoryAction.h>
#include <move_rt/TaskParamUpdate.h>
#include <move_rt/functions.hpp>

#include <trajectory_msgs/JointTrajectory.h> //visualization_msgs::Marker marker
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group/move_group_context.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <actionlib/server/action_server.h>

using namespace std;
using namespace Eigen;

/// @brief PriorityLevel is abstact base for all task controllers

class PriorityLevel {
public:
  /**
   * Constructor of the abstact base for all task controllers
   * could take one parameter
   * @param nodehandle is a pointer to a ros nodeHandle
   */
  PriorityLevel(ros::NodeHandle &nodehandle);

  /**
   * Virtual destructor of the task base class.
   */
  ~PriorityLevel();

  /**
   * update: function that finds the joint velocities (dq) of the given task
   * controller.
   */
  void update(bool);

  /**
   * update_task: update the parameters of the current task controller.
   */
  void update_task();

  /**
   * show: print the parameters of the current task controller.
   */
  void show();

  /**
   * control: default control loop of the task
   */
  virtual void control();

  /**
   * updateJk: Pure Virtual Function to update the Jacobian of the current task
   * controller.
   */
  virtual void updateJk(){};

  /**
   * updateA: Pure Virtual Function to update the selection matrix of the
   * current task controller.
   */
  virtual void updateA(){};

  /**
   * updateConstraints: Pure Virtual Function to update the constraints of the
   * current task controller.
   */
  virtual void updateConstraints(){};

  /**
   * get_task: Pure Virtual Function to return the current task controller name.
   */
  virtual string get_task() = 0;

  /**
   * get_JK: Pure Virtual Function to return the task Jacobian.
   */
  virtual Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
  get_JK() = 0;

  /**
   * serviceSetEnable: Callback function to enable and disable the task.
   */
  bool serviceSetEnable(move_rt::TaskParamUpdate::Request &req,
                        move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_set_enable;

  /**
   * callbackSetK: Callback function to update the task gain diagonal matrix.
   */
  bool serviceSetK(move_rt::TaskParamUpdate::Request &req,
                   move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_ee_K_setK;

  /**
   * serviceSetxm: Service function to update the task constraints.
   */
  bool serviceSetxm(move_rt::TaskParamUpdate::Request &req,
                    move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_ee_xm;

  /**
   * serviceSetb: Service function to update the task activation range.
   */
  bool serviceSetb(move_rt::TaskParamUpdate::Request &req,
                   move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_ee_b;

  /**
   * serviceSetA: Service function to update the task activation matrix.
   */
  bool serviceSetA(move_rt::TaskParamUpdate::Request &req,
                   move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_ee_A;

  /**
   * serviceGetA: Service function to get the task activation matrix.
   */
  bool serviceGetA(move_rt::TaskParamUpdate::Request &req,
                   move_rt::TaskParamUpdate::Response &res);
  ros::ServiceServer sub_get_A;

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A, K;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> xd, dxd, x, dx, xm, b, qK;

protected:
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> JK;

  string name_task;
  ros::NodeHandle _nodehandle;

  int m;

  bool initialized;

  bool enabled;

  double cycleHz = 25;
};

#endif // PRIORITYLEVEL_H

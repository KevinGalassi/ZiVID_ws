#ifndef CONTROLMANAGER_H
#define CONTROLMANAGER_H

#include <move_rt/dqminimizer.hpp>
#include <move_rt/eeposition.hpp>
#include <move_rt/emergencystop.hpp>
#include <move_rt/functions.hpp>
#include <move_rt/jointlimiter.hpp>
#include <move_rt/jointposition.hpp>
#include <move_rt/prioritylevel.hpp>
#include <move_rt/robotsafety.hpp>
#include <move_rt/elbowcontrol.hpp>
#include <move_rt/singularityavoidance.hpp>

class PriorityLevel;

/// @brief ControlManager class manages the task hierarchy. It first reads all
/// the control in the yaml file and create their own instance. It updates all
/// the variables of the robot (joint positions,etc) and computes the joint
/// velocities based on task priority control and sends them to the robot's
/// joints.
class ControlManager {
public:
  /**
   * Constructor that set all required controllers, their parameters.
   *
   * @param nodehandle: a pointer to a ros nodeHandle
   */
  ControlManager(ros::NodeHandle &nodehandle);

  /**
   * Destructor of the class.
   */
  ~ControlManager();

  /**
   * update: update all the required variable of the task controllers from the
   * current state of the robot.
   */
  void update();

  /**
   * show: print the parameters of the current task controller.
   */
  void show();

  // friend class PriorityLevel;

private:
  Functions f;

  void update_prioritylevel(PriorityLevel *task);
  void velocitysaturation();

  // Define some helper methods to encapsulate the gory details of initializing
  // subscribers, publishers
  void initializeSubscribers(); // we will define some helper methods to
                                // encapsulate the gory details of initializing
                                // subscribers, publishers and services
  void initializePublishers();

  // Call Back function: read the joint positions of the robot
  void callbackJState(const sensor_msgs::JointState::ConstPtr &msg);
  sensor_msgs::JointState joint_state;

  // level_manager: vector of pointers to the different robotic tasks. Higher
  // task is the first task in the vector (index 0), etc...
  std::vector<PriorityLevel *> level_manager;

  // n: number of degree of freedom of the robot
  int n;

  string controller_topic, joint_state_topic;
  std::vector<std::string> ordered_joints;

  // Task priority controller parameters
  double eta;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> q, dq;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> dqd, dqd_old;
  Eigen::Matrix<long double, Eigen::Dynamic, 1> dqdmax, dqdmin;
  Eigen::Matrix<long double, Eigen::Dynamic, 1> dqdmax_tmp, dqdmin_tmp;

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Q;

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> M;

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> I, Qk;
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> JkQk_1, wnpJkQk_1Q,
      wnpJkQk_1I, Wk, Tk;

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> JK;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> alpha, delta_dqd;

  long double a;

  double cycleHz = 25;
  ros::NodeHandle _nodehandle;

  ros::Publisher pub_velocity_joints;
  ros::Subscriber sub_jacobian_states;

  std_msgs::Float64MultiArray
      trajMsg; /*it's defined by the arrayed version of the matrix and the
                  layout of the matrix*/

  bool initialized;
};

#endif // PRIORITYLEVEL_H

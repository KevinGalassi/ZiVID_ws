#ifndef ELBOWPOSITION_H
#define ELBOWPOSITION_H

#include <move_rt/prioritylevel.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

class ElbowPosition : public PriorityLevel {
public:
  ElbowPosition(ros::NodeHandle &nodehandle);
  ~ElbowPosition();

  void control();

  void updateJk();
  void updateConstraints();

  string get_task();

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };

private:
  string base_link, ee_link;

  ros::Subscriber sub_elbow_mode;
  void callbackelbowmode(const std_msgs::Int16::ConstPtr &msg);

  int elbowmode;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener; //(tfBuffer);
  geometry_msgs::TransformStamped transform_above;

  bool valid_JK;

  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position;

  std::string group;
  std::string model;

  robot_model_loader::RobotModelLoader *robot_model_loader_ptr;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::JointModelGroup *joint_model_group;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::string PLANNING_SCENE_SERVICE;
};

#endif // ELBOWPOSITION_H

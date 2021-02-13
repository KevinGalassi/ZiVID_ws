#include <move_rt/elbowcontrol.hpp>

ElbowPosition::ElbowPosition(ros::NodeHandle &nodehandle)
    : PriorityLevel(nodehandle) {
  ROS_DEBUG_STREAM("enter ElbowPosition contructor ");

  PriorityLevel::update_task();

  sub_elbow_mode = nodehandle.subscribe<std_msgs::Int16>(
      "set_elbow_mode", 1, &ElbowPosition::callbackelbowmode, this);

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  elbowmode = 3;

  // getting these two as arguments from the launch file
  ROS_DEBUG_STREAM("ElbowPosition::ElbowPosition: getting model ");

  model = "robot_description";

  ROS_DEBUG_STREAM("ElbowPosition::ElbowPosition: I got model ");

  nodehandle.getParam(get_task() + "/base_link", base_link);
  nodehandle.getParam(get_task() + "/ee_link", ee_link);
  nodehandle.getParam(get_task() + "/group", group);

  JK.setZero();

  ROS_DEBUG_STREAM("ElbowPosition::ElbowPosition: I got parameters ");

  ROS_DEBUG_STREAM("ElbowPosition::ElbowPosition: loading model " << model);

  robot_model_loader::RobotModelLoader robot_model_loader(model);

  ROS_DEBUG_STREAM("ElbowPosition::ElbowPosition: "
                   "robot_model_loader::RobotModelLoader called ");

  kinematic_model = robot_model_loader.getModel();

  ROS_DEBUG_STREAM(
      "ElbowPosition::ElbowPosition: robot_model::RobotModelPtr called ");

  joint_model_group = kinematic_model->getJointModelGroup(group); //???

  ROS_DEBUG_STREAM(
      "ElbowPosition::ElbowPosition: "
      "kinematic_model->getJointModelGroup(group) called for group "
      << group);

  planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(model);

  PLANNING_SCENE_SERVICE = "get_planning_scene";

  reference_point_position << 0.0, 0.0, 0.0;

  updateConstraints();

  A.setIdentity();

  valid_JK = false;

  ROS_DEBUG_STREAM("exit ElbowPosition contructor ");
}

ElbowPosition::~ElbowPosition() {}

void ElbowPosition::control() {
  if ((elbowmode == 1) || (elbowmode == 2)) // keep the elbow up
  {
    // std::cout<<elbowmode<<"\n"<<std::endl;
    dx(0, 0) = K(0, 0) * (xd(0, 0) - x(0, 0));
  }
  //       else if(elbowmode==2)  // keep the elbow down
  //        {
  //          //std::cout<<elbowmode<<"\n"<<std::endl;
  //          dx(0,0)=K(0,0)*(xd(0,0) - x(0,0));
  //        }
  else if (elbowmode == 3) // elbow free
  {
    // std::cout<<elbowmode<<"\n"<<std::endl;
    dx(0, 0) = 0; // it is automatically zero since xd = x
  }
}

void ElbowPosition::updateJk() {

  ROS_DEBUG_STREAM("ElbowPosition::updateJk: entering ");

  valid_JK = planning_scene_monitor_->requestPlanningSceneState(
      PLANNING_SCENE_SERVICE);

  ROS_DEBUG_STREAM(
      "ElbowPosition::updateJk: requestPlanningSceneState called ");

  planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
  ps->getCurrentStateNonConst().update();

  robot_state::RobotState new_kinematic_state = ps->getCurrentStateNonConst();

  new_kinematic_state.getJacobian(new_kinematic_state.getJointModelGroup(group),
                                  new_kinematic_state.getLinkModel(ee_link),
                                  reference_point_position, jacobian);

  if (jacobian.rows() == 0 || jacobian.cols() == 0)
    return;

  for (int j = 0; j < jacobian.cols(); j++) {

    ElbowPosition::JK(0, j) = jacobian(2, j);

    ROS_DEBUG_STREAM("ElbowPosition::updateJk: jacobian("
                     << 0 << "," << j << ") = " << jacobian(0, j));

    ROS_DEBUG_STREAM("ElbowPosition::updateJk: JK(" << 0 << "," << j
                                                    << ") = " << JK(0, j));
  }

  ROS_DEBUG_STREAM("ElbowPosition::updateJk: finished ");
}

void ElbowPosition::updateConstraints() {
  ROS_DEBUG_STREAM("ElbowPosition::updateConstraints: entering ");

  try {
    transform_above = tfBuffer.lookupTransform(base_link, ee_link, ros::Time(0),
                                               ros::Duration(1000 / 200));
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  x(0, 0) = transform_above.transform.translation.z;

  if (elbowmode == 1)
    xd(0, 0) = 0.15;
  else if (elbowmode == 2)
    xd(0, 0) = -0.15;
  else if (elbowmode == 3)
    xd(0, 0) = x(0, 0);
}

string ElbowPosition::get_task() { return "ElbowPosition"; }

void ElbowPosition::callbackelbowmode(const std_msgs::Int16::ConstPtr &msg) {
  elbowmode = msg->data;
  ROS_INFO_STREAM("ElbowPosition::callbackelbowmode: elbow mode set to "
                  << msg->data);
}

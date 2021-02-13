#include <move_rt/eeposition.hpp>

Quaterniond quaternion_difference(const Quaterniond &a, const Quaterniond &b) {
  Quaterniond inv = a;
  inv.inverse();
  return inv * b;
}

Quaterniond quaternion_negation(Quaterniond v) {
  Quaterniond res;
  res.w() = -v.w();
  res.x() = -v.x();
  res.y() = -v.y();
  res.z() = -v.z();

  return res;
}

Quaterniond scalar_product(Quaterniond v, double t) {
  Quaterniond res;
  res.w() = t * v.w();
  res.x() = t * v.x();
  res.y() = t * v.y();
  res.z() = t * v.z();

  return res;
}

Quaterniond quaternion_minus(const Quaterniond &v1, const Quaterniond &v0) {
  Quaterniond vdiff;
  vdiff.w() = v1.w() - v0.w();
  vdiff.x() = v1.x() - v0.x();
  vdiff.y() = v1.y() - v0.y();
  vdiff.z() = v1.z() - v0.z();

  return vdiff;
}

Quaterniond quaternion_plus(const Quaterniond &v1, const Quaterniond &v0) {
  Quaterniond vadd;
  vadd.w() = v1.w() + v0.w();
  vadd.x() = v1.x() + v0.x();
  vadd.y() = v1.y() + v0.y();
  vadd.z() = v1.z() + v0.z();

  return vadd;
}

Quaterniond my_slerp(Quaterniond v0, Quaterniond v1, double t) {
  // Only unit quaternions are valid rotations.
  // Normalize to avoid undefined behavior.
  v0.normalize();
  v1.normalize();

  // Compute the cosine of the angle between the two vectors.
  double dot = v0.dot(v1);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that v1 and -v1 are equivalent when
  // the negation is applied to all four components. Fix by
  // reversing one quaternion.
  if (dot < 0.0f) {
    v1 = quaternion_negation(v1);
    dot = -dot;
  }

  Quaterniond vdiff = quaternion_minus(v1, v0);

  const double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD) {
    // If the inputs are too close for comfort, linearly interpolate
    // and normalize the result.

    Quaterniond result = scalar_product(vdiff, t);
    result.normalize();
    return result;
  }

  // Since dot is in range [0, DOT_THRESHOLD], acos is safe
  double theta_0 = acos(dot);        // theta_0 = angle between input vectors
  double theta = theta_0 * t;        // theta = angle between v0 and result
  double sin_theta = sin(theta);     // compute this value only once
  double sin_theta_0 = sin(theta_0); // compute this value only once

  double s0 =
      cos(theta) -
      dot * sin_theta / sin_theta_0; // == sin(theta_0 - theta) / sin(theta_0)
  double s1 = sin_theta / sin_theta_0;

  return quaternion_plus(scalar_product(v0, s0), scalar_product(v1, s1));
}

EePosition::EePosition(
    ros::NodeHandle &nodehandle,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> &_JK)
    : action_server_(nodehandle, "ee_execute_trajectory",
                     boost::bind(&EePosition::goalCB, this, _1),
                     boost::bind(&EePosition::cancelCB, this, _1), false),
      has_active_goal_(false), PriorityLevel(nodehandle), JK(_JK) {
  ROS_DEBUG_STREAM("enter EePosition contructor ");

  PriorityLevel::update_task();

  string name = get_task() + "/clearTrajectory";
  sub_clear_trajectory = nodehandle.advertiseService(
      name, &EePosition::serviceClearTrajectory, this);

  name = get_task() + "/setToolFrame";
  service_set_tool_frame =
      nodehandle.advertiseService(name, &EePosition::serviceSetToolFrame, this);

  sub_ee_setpoint = nodehandle.subscribe<std_msgs::Float64MultiArray>(
      "ee_setpoint", 1, &EePosition::callbackEeSetpoint, this);

  sub_wrench = nodehandle.subscribe<geometry_msgs::WrenchStamped>(
      "wrench", 1, &EePosition::callbackWrench, this);

  ee_error_pub =
      nodehandle.advertise<std_msgs::Float64MultiArray>("ee_error", 1);
  ee_position_pub =
      nodehandle.advertise<std_msgs::Float64MultiArray>("ee_position", 1);
  ee_trajectory_pub =
      nodehandle.advertise<std_msgs::Float64MultiArray>("ee_trajectory", 1);

  errMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  errMsg.layout.data_offset = 0;
  errMsg.layout.dim[0].label = "error";
  errMsg.layout.dim[0].size = 7;
  errMsg.layout.dim[0].stride = 0;

  posMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  posMsg.layout.data_offset = 0;
  posMsg.layout.dim[0].label = "position";
  posMsg.layout.dim[0].size = 7;
  posMsg.layout.dim[0].stride = 0;

  trajMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  trajMsg.layout.data_offset = 0;
  trajMsg.layout.dim[0].label = "trajectory";
  trajMsg.layout.dim[0].size = 7;
  trajMsg.layout.dim[0].stride = 0;

  traj_length = 0;

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  x.resize(7, 1);

  xd.resize(7, 1);

  dxd.resize(6, 1);
  dxd.setZero();

  // getting these two as arguments from the launch file
  ROS_DEBUG_STREAM("EePosition::EePosition: getting model ");

  _nodehandle.param<std::string>(get_task() + "robot_description", model,
                                 "robot_description");

  ROS_DEBUG_STREAM("EePosition::EePosition: I got model ");

  _nodehandle.getParam("cycleHz", cycleHz);

  _nodehandle.getParam(get_task() + "/base_link", base_link);
  _nodehandle.getParam(get_task() + "/ee_link", ee_link);
  if (_nodehandle.hasParam(get_task() + "/tf_prefix")){
    _nodehandle.getParam(get_task() + "/tf_prefix", tf_prefix);
    if(tf_prefix.find_last_not_of("/")) tf_prefix.push_back('/');    
  }

  _nodehandle.getParam(get_task() + "/group", group);

  ROS_DEBUG_STREAM("EePosition::EePosition: I got parameters ");

  planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(model);

  _nodehandle.param<std::string>(get_task() + "/planning_scene",
                                 PLANNING_SCENE_SERVICE, "get_planning_scene");

  reference_point_position << 0.0, 0.0, 0.0;

  quat.resize(4, 1);

  obj.resize(4, 1);

  E.resize(3, 4);

  x_diff.resize(6, 1);

  action_server_.start();

  A.setZero();
  enabled = false;

  updateConstraints();

  xd = x;

  // Kp << 0.0, 0.0, 0.0001;
  Kp << 0.0, 0.0, 0.0;

  Ka << -0.2, -0.2, -0.5;
  // Ka << 0.0, 0.0, 0.0;

  disp.setZero();

  valid_JK = false;

  ROS_DEBUG_STREAM("exit EePosition contructor ");

  visual_tools = new moveit_visual_tools::MoveItVisualTools(base_link);
  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
}

EePosition::~EePosition() { delete visual_tools; }

void EePosition::control() {

  quat(0, 0) = x(3, 0);
  quat(1, 0) = x(4, 0);
  quat(2, 0) = x(5, 0);
  quat(3, 0) = x(6, 0);

  obj(0, 0) = xd(3, 0);
  obj(1, 0) = xd(4, 0);
  obj(2, 0) = xd(5, 0);
  obj(3, 0) = xd(6, 0);

  double dot = quat.dot(obj);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that v1 and -v1 are equivalent when
  // the negation is applied to all four components. Fix by
  // reversing one quaternion.
  if (dot < 0.0f)
    obj = -obj;

  E << -quat(1, 0), quat(0, 0), -quat(3, 0), quat(2, 0), -quat(2, 0),
      quat(3, 0), quat(0, 0), -quat(1, 0), -quat(3, 0), -quat(2, 0), quat(1, 0),
      quat(0, 0);

  w = 2 * cycleHz * E * (obj - quat);

  double w_norm = w.norm();

  for (int i = 0; i < 3; i++)
    x_diff(i, 0) = xd(i) - x(i);
  for (int i = 3; i < 6; i++)
    x_diff(i, 0) = w(i - 3);

  if (valid_JK)
    dx = K * x_diff + dxd;
  else
    dx.setZero();

  errMsg.data.clear();
  posMsg.data.clear();
  trajMsg.data.clear();

  for (int i = 0; i < 6; i++) {
    errMsg.data.push_back(x_diff(i));
  }

  for (int i = 0; i < 7; i++) {
    posMsg.data.push_back(x(i));
    trajMsg.data.push_back(xd(i));
  }

  errMsg.data.push_back(x_diff.squaredNorm());

  ee_error_pub.publish(errMsg);
  ee_position_pub.publish(posMsg);
  ee_trajectory_pub.publish(trajMsg);
}

void EePosition::updateJk() {

  ROS_DEBUG_STREAM("EePosition::updateJk: calling requestPlanningSceneState");

  valid_JK = planning_scene_monitor_->requestPlanningSceneState(
      PLANNING_SCENE_SERVICE);

  ROS_DEBUG_STREAM("EePosition::updateJk: requestPlanningSceneState called ");

  planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);

  ps->getCurrentStateNonConst().update();

  robot_state::RobotState new_kinematic_state = ps->getCurrentStateNonConst();

  new_kinematic_state.getJacobian(new_kinematic_state.getJointModelGroup(group),
                                  new_kinematic_state.getLinkModel(ee_link),
                                  reference_point_position, jacobian);

  if (jacobian.rows() == 0 || jacobian.cols() == 0)
    return;

  for (int i = 0; i < jacobian.rows(); i++)
    for (int j = 0; j < jacobian.cols(); j++) {

      EePosition::JK(i, j) = jacobian(i, j);

      ROS_DEBUG_STREAM("EePosition::updateJk: jacobian("
                       << i << "," << j << ") = " << jacobian(i, j));

      ROS_DEBUG_STREAM("EePosition::updateJk: JK(" << i << "," << j
                                                   << ") = " << JK(i, j));
    }
}

void EePosition::updateConstraints() {
  double pos_error = 0, orient_error = 0;

  try {
    transform_above = tfBuffer.lookupTransform(base_link, tf_prefix + ee_link, ros::Time(0),
                                               ros::Duration(1000 / 200));
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  x(0, 0) = transform_above.transform.translation.x;
  x(1, 0) = transform_above.transform.translation.y;
  x(2, 0) = transform_above.transform.translation.z;
  x(3, 0) = transform_above.transform.rotation.w;
  x(4, 0) = transform_above.transform.rotation.x;
  x(5, 0) = transform_above.transform.rotation.y;
  x(6, 0) = transform_above.transform.rotation.z;

  if (!traj_points.empty()) {
    traj_point = traj_points.front();

    for (int i = 0; i < 7; i++)
      xd(i, 0) = traj_point(i, 0);

    for (int j = 0; j < 6; j++)
      dxd(j, 0) = traj_point(j + 7, 0);

    for (int i = 0; i < 3; i++)
      pos_error += pow(xd(i) - x(i), 2);

    for (int i = 3; i < 7; i++)
      orient_error += pow(xd(i) - x(i), 2);

    // if(  sqrt(pos_error)<0.1 && sqrt(orient_error)<0.1 )
    traj_points.pop();
  }

  if (has_active_goal_) {
    feedback_.percent_complete =
        (1.0 - (traj_points.size() / (double)traj_length)) * 100;
    active_goal_.publishFeedback(feedback_);

    if (traj_points.empty()) {

      if (x_diff.squaredNorm() > active_goal_.getGoal()->ee_error_th) {
        result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
        active_goal_.setCanceled(result_);
        has_active_goal_ = false;
      } else {
        result_.error_code = result_.SUCCESSFUL;
        active_goal_.setSucceeded(result_);
        has_active_goal_ = false;
      }
    }
  }
}

string EePosition::get_task() { return "EePosition"; }

void EePosition::callbackEeSetpoint(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  Eigen::Matrix<long double, 8, 1> tmp_xd;
  Eigen::Matrix<long double, 7, 1> init_point;
  Eigen::Matrix<long double, 13, 1> traj_point;

  double cycleHz;

  _nodehandle.getParam("cycleHz", cycleHz);

  if (!traj_points.empty()) {
    traj_point = traj_points.back();
    for (int i = 0; i < 7; i++)
      init_point(i, 0) = traj_point(i, 0);
  } else
    for (int i = 0; i < 7; i++)
      init_point(i, 0) = x(i, 0);

  if (msg->data[7] <= 0.0) {
    ROS_INFO_STREAM("data [7] must be greater than 0.0");
    return;
  }

  for (int i = 0; i < 8; i++) {
    tmp_xd(i, 0) = msg->data[i];
    // ROS_INFO_STREAM("new setpoint " << i << " " << tmp_xd(i,0));
  }

  Quaterniond qa(init_point(3, 0), init_point(4, 0), init_point(5, 0),
                 init_point(6, 0)),
      qb(tmp_xd(3, 0), tmp_xd(4, 0), tmp_xd(5, 0), tmp_xd(6, 0)), qres;
  // initialize qa, qb;

  qa.normalize();
  qb.normalize();

  double dot = qa.dot(qb);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that v1 and -v1 are equivalent when
  // the negation is applied to all four components. Fix by
  // reversing one quaternion.
  if (dot < 0.0f)
    qb = quaternion_negation(qb);

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> E, w;
  E.resize(3, 4);
  E << -qa.x(), qa.w(), -qa.z(), qa.y(), -qa.y(), qa.z(), qa.w(), -qa.x(),
      -qa.z(), -qa.y(), qa.x(), qa.w();

  Eigen::Matrix<long double, 4, 1> qdiff(qb.w() - qa.w(), qb.x() - qa.x(),
                                         qb.y() - qa.y(), qb.z() - qa.z());

  w = 2 / tmp_xd(7, 0) * E * qdiff;

  for (int j = 0; j < 3; j++)
    traj_point(j + 7, 0) =
        (tmp_xd(j, 0) - init_point(j, 0)) / (2 * tmp_xd(7, 0));

  for (int j = 0; j < 3; j++)
    traj_point(j + 10, 0) = w(j, 0);

  all_goals.clear();
  geometry_msgs::Pose target_goal;

  for (int i = 0; i <= tmp_xd(7, 0) * cycleHz; i++) {

    for (int j = 0; j < 3; j++)
      traj_point(j, 0) = init_point(j, 0) + (tmp_xd(j, 0) - init_point(j, 0)) *
                                                (i / (tmp_xd(7, 0) * cycleHz));

    qres = qa.slerp(i / (tmp_xd(7, 0) * cycleHz), qb);

    traj_point(3, 0) = qres.w();
    traj_point(4, 0) = qres.x();
    traj_point(5, 0) = qres.y();
    traj_point(6, 0) = qres.z();

    traj_points.push(traj_point);

    target_goal.orientation.x = traj_point(4, 0);
    target_goal.orientation.y = traj_point(5, 0);
    target_goal.orientation.z = traj_point(6, 0);
    target_goal.orientation.w = traj_point(3, 0);
    target_goal.position.x = traj_point(0, 0);
    target_goal.position.y = traj_point(1, 0);
    target_goal.position.z = traj_point(2, 0);
    all_goals.push_back(target_goal);
  }

  // set velocity to zero at the last point
  for (int j = 0; j < 6; j++)
    traj_point(j + 7, 0) = 0.0;

  traj_points.push(traj_point);

  traj_length = traj_points.size();

  target_goal.orientation.x = traj_point(4, 0);
  target_goal.orientation.y = traj_point(5, 0);
  target_goal.orientation.z = traj_point(6, 0);
  target_goal.orientation.w = traj_point(3, 0);
  target_goal.position.x = traj_point(0, 0);
  target_goal.position.y = traj_point(1, 0);
  target_goal.position.z = traj_point(2, 0);
  all_goals.push_back(target_goal);

  ROS_INFO_STREAM("callbackEeSetpoint: tmp_xd = \n" << tmp_xd);

  for (int i = 0; i < all_goals.size(); i++)
    visual_tools->publishAxisLabeled(all_goals[i], "target_pose",
                                     rvt::XXXSMALL);

  visual_tools->trigger();
}

void EePosition::callbackWrench(
    const geometry_msgs::WrenchStamped::ConstPtr &msg) {

  double force_threshold = 15.0;

  force(0) = -msg->wrench.force.x;
  force(1) = -msg->wrench.force.y;
  force(2) = msg->wrench.force.z;
  torque(0) = -msg->wrench.torque.x;
  torque(1) = -msg->wrench.torque.y;
  torque(2) = msg->wrench.torque.z;

  for (int i = 0; i < 3; i++) {
    // disp(i)=0;
    if (force(i) > force_threshold)
      disp(i) += Kp(i) * (force(i) - force_threshold);
    else if (force(i) < -force_threshold)
      disp(i) += Kp(i) * (force(i) + force_threshold);
    // else if(abs(force(i))<force_threshold)
    disp(i) -= 0.1 * disp(i);

    //   x(i,0) += disp(i);
  }
}

void EePosition::goalCB(GoalHandle gh) {

  ROS_INFO("EePosition::goalCB: goal received");

  // Ensures that the joints in the goal match the joints we are commanding.
  if (gh.getGoal()->trajectory_name.length() == 0) {
    ROS_ERROR("EePosition::goalCB: No trajectory points specified");
    gh.setRejected();
    return;
  }

  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;

  if (!_nodehandle.hasParam(active_goal_.getGoal()->trajectory_name)) {
    ROS_INFO_STREAM("EePosition::goalCB: trajectory "
                    << active_goal_.getGoal()->trajectory_name
                    << " does not exist in parameter server");
    return;
  } else {

    XmlRpc::XmlRpcValue waypoints_vector;

    _nodehandle.getParam(active_goal_.getGoal()->trajectory_name,
                         waypoints_vector);

    if (waypoints_vector.getType() == XmlRpc::XmlRpcValue::Type::TypeArray &&
        waypoints_vector.size() > 0) {
      ROS_INFO_STREAM("EePosition::goalCB: trajectory "
                      << active_goal_.getGoal()->trajectory_name
                      << " loaded with " << waypoints_vector.size()
                      << " points");
      // waypoints_vector[0] is a 'TypeArray' aka vector
    } else {
      ROS_INFO("EePosition::goalCB: trajectory is empty");
      return;
    }

    boost::shared_ptr<std_msgs::Float64MultiArray> trajectory_point(
        new std_msgs::Float64MultiArray());

    trajectory_point->layout.dim.push_back(
        std_msgs::MultiArrayDimension()); // one structure MultiArrayDimension
                                          // per dimension
    trajectory_point->layout.data_offset = 0;
    trajectory_point->layout.dim[0].label = "";
    trajectory_point->layout.dim[0].size = 8;
    trajectory_point->layout.dim[0].stride = 0;

    std::vector<std::string> fields{"pos_x",    "pos_y",    "pos_z",
                                    "orient_w", "orient_x", "orient_y",
                                    "orient_z", "time"};

    for (int i = 0; i < waypoints_vector.size(); i++) {
      trajectory_point->data.clear();

      if (waypoints_vector[i].getType() ==
          XmlRpc::XmlRpcValue::Type::TypeStruct) {
        for (int j = 0; j < fields.size(); j++) {
          if (waypoints_vector[i].hasMember(fields[j])) {
            trajectory_point->data.push_back(
                double(waypoints_vector[i][fields[j]]));
          } else {
            ROS_INFO(
                "EePosition::goalCB: trajectory waypoint[%d] has no member %s",
                i, fields[j].c_str());
            return;
          }
        }
      } else {
        ROS_INFO(
            "EePosition::goalCB: trajectory waypoint[%d] type is not a Struct",
            i);
        return;
      }

      callbackEeSetpoint(trajectory_point);
    }

    ROS_INFO("EePosition::goalCB: trajectory is sent");
  }
}

void EePosition::cancelCB(GoalHandle gh) {

  ROS_INFO("EePosition::cancelCB: cancellation request received");

  // Cancels the currently active goal.
  if (has_active_goal_) {
    // clear the trajectory.
    clear_traj_points();
    traj_length = 0;
    all_goals.clear();

    // Marks the current goal as canceled.
    result_.error_code = result_.INVALID_GOAL;
    active_goal_.setCanceled(result_);
    has_active_goal_ = false;
  }
}

bool EePosition::serviceClearTrajectory(
    move_rt::TaskParamUpdate::Request &req,
    move_rt::TaskParamUpdate::Response &res) {
  visual_tools->deleteAllMarkers();
  visual_tools->trigger();

  ROS_INFO_STREAM(get_task()
                  << "::serviceClearTrajectory: trajectory cleaned\n");

  res.result = true;
  return true;
}

bool EePosition::serviceSetToolFrame(move_rt::UpdateFrame::Request &req,
                                     move_rt::UpdateFrame::Response &res) {

  if (traj_points.empty()) {
    ee_link = req.frame;

    updateConstraints();

    xd = x;

    ROS_INFO_STREAM(get_task() << "::serviceSetToolFrame: tool frame set to "
                               << req.frame);

    res.result = true;
    return true;
  } else {
    ROS_INFO_STREAM(get_task() << "::serviceSetToolFrame: trajectory not "
                                  "empty, tool frame set aborted");

    res.result = false;
    return false;
  }
}

void EePosition::show() {
  PriorityLevel::show();

  std::cout << "w= " << w << "\n";
  std::cout << "quat= " << quat << "\n";
  std::cout << "obj= " << obj << "\n";

  ROS_INFO_STREAM("x: [");
  for (int i = 0; i < x.size(); i++)
    if (i < x.size() - 1)
      ROS_INFO_STREAM(x(i, 0) << ", ");
    else {
      ROS_INFO_STREAM(x(i, 0) << "], ");
    }

  ROS_INFO_STREAM("xd: [");
  for (int i = 0; i < xd.size(); i++)
    if (i < xd.size() - 1)
      ROS_INFO_STREAM(xd(i, 0) << ", ");
    else {
      ROS_INFO_STREAM(xd(i, 0) << "], ");
    }

  ROS_INFO_STREAM("dx: [");
  for (int i = 0; i < dx.size(); i++)
    if (i < dx.size() - 1)
      ROS_INFO_STREAM(dx(i, 0) << ", ");
    else {
      ROS_INFO_STREAM(dx(i, 0) << "], ");
    }

  ROS_INFO_STREAM("dxd: [");
  for (int i = 0; i < dxd.size(); i++)
    if (i < dxd.size() - 1)
      ROS_INFO_STREAM(dxd(i, 0) << ", ");
    else {
      ROS_INFO_STREAM(dxd(i, 0) << "], ");
    }
}

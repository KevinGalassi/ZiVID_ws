// this header incorporates all the necessary #include files and defines the
// class "ExampleRosClass"
#include <move_rt/jointposition.hpp>

static bool setsEqual(const std::vector<std::string> &a,
                      const std::vector<std::string> &b) {
  if (a.size() != b.size())
    return false;

  for (size_t i = 0; i < a.size(); ++i) {
    if (count(b.begin(), b.end(), a[i]) != 1)
      return false;
  }
  for (size_t i = 0; i < b.size(); ++i) {
    if (count(a.begin(), a.end(), b[i]) != 1)
      return false;
  }

  return true;
}

JointPosition::JointPosition(ros::NodeHandle &n,
                             Eigen::Matrix<long double, Eigen::Dynamic, 1> &_q,
                             sensor_msgs::JointState &joint_state)
    : action_server_(n, "follow_joint_trajectory",
                     boost::bind(&JointPosition::goalCB, this, _1),
                     boost::bind(&JointPosition::cancelCB, this, _1), false),
     JS_action_server_(n, "simple_joint_trajectory",
                     boost::bind(&JointPosition::JS_goalCB, this, _1),
                     boost::bind(&JointPosition::JS_cancelCB, this, _1), false),
       has_active_goal_(false), JS_has_active_goal_(false), PriorityLevel(n), q(_q),
      joint_state_(joint_state) {
  using namespace XmlRpc;

  ROS_DEBUG_STREAM("JointPosition::JointPosition: enter contructor ");

  PriorityLevel::update_task();

  string controller_topic;

  n.getParam("controller_topic", controller_topic);

  pub_controller_state =
      n.advertise<control_msgs::JointTrajectoryControllerState>(
          controller_topic + "/state", 1);
      
  sub_joint_setpoint = n.subscribe<std_msgs::Float64MultiArray>(
      "joint_setpoint", 1, &JointPosition::callbackJointSetpoint,
      this);

  trajectory_controller_state.header.frame_id = "world";
  trajectory_controller_state.header.stamp = ros::Time::now();

  pub_controller_state.publish(trajectory_controller_state);

  ROS_INFO("JointPosition::JointPosition: Initializing Action Server");

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!n.getParam(controller_topic + "/joints", joint_names)) {
    //ROS_WARN("No joints given. (namespace: %s)", n.getNamespace().c_str());
    _nodehandle.getParam("ordered_joints", joint_names);
  }

  ROS_DEBUG_STREAM("JointPosition::JointPosition: joint name passed");

  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("Malformed joint specification.  (namespace: %s)",
              n.getNamespace().c_str());
    exit(1);
  }

  ROS_DEBUG_STREAM("JointPosition::JointPosition: joint types passed");

  for (int i = 0; i < joint_names.size(); ++i) {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString) {
      ROS_FATAL(
          "Array of joint names should contain all strings.  (namespace: %s)",
          n.getNamespace().c_str());
      exit(1);
    }

    joint_names_.push_back((std::string)name_value);
  }

  ROS_DEBUG_STREAM("JointPosition::JointPosition: joint values passed");

  n.param("constraints/goal_time", goal_time_constraint_, 0.0);

  // Gets the constraints for each joint.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    std::string ns = std::string("constraints/") + joint_names_[i];
    double g, t;
    n.param(ns + "/goal", g, -1.0);
    n.param(ns + "/trajectory", t, -1.0);
    goal_constraints_[joint_names_[i]] = g;
    trajectory_constraints_[joint_names_[i]] = t;
  }
  n.param("/constraints/stopped_velocity_tolerance",
          stopped_velocity_tolerance_, 0.1);

  ROS_DEBUG_STREAM("JointPosition::JointPosition: joint constraints passed");

  sub_controller_state_ = n.subscribe(controller_topic + "/state", 1,
                                      &JointPosition::controllerStateCB, this);

  watchdog_timer_ =
      n.createTimer(ros::Duration(1.0), &JointPosition::watchdog, this);

  ros::Time started_waiting_for_controller = ros::Time::now();

  ROS_DEBUG_STREAM("JointPosition::JointPosition: before while");

  while (ros::ok() && last_controller_state_) //! last_controller_state_
  {
    ros::spinOnce();
    if (started_waiting_for_controller != ros::Time(0) &&
        ros::Time::now() >
            started_waiting_for_controller + ros::Duration(30.0)) {
      ROS_WARN(
          "Waited for the controller for 30 seconds, but it never showed up.");
      started_waiting_for_controller = ros::Time(0);
    }
    ros::Duration(0.1).sleep();
  }

  ROS_DEBUG_STREAM("JointPosition::JointPosition: after while");

  action_server_.start();

  ROS_DEBUG_STREAM("JointPosition::JointPosition: action server started");

  ROS_DEBUG_STREAM("JointPosition::JointPosition: Activating Joint Simple Trajectory Action Server");

  JS_action_server_.start();

  traj_length = 0;
  
  ROS_DEBUG_STREAM("JointPosition::JointPosition: Joint Simple Trajectory Action Server started");
  
  JK.setIdentity();

  A.setZero();

  trajectory_controller_state.joint_names = joint_names_;

  trajectory_index = 0;
  
  q_err.resize(m, 1);

  trajectory_controller_state.desired.positions.resize(m);
  trajectory_controller_state.desired.velocities.resize(m);
  trajectory_controller_state.desired.effort.resize(m);

  trajectory_controller_state.actual.positions.resize(m);
  trajectory_controller_state.actual.velocities.resize(m);
  trajectory_controller_state.actual.effort.resize(m);

  trajectory_controller_state.error.positions.resize(m);
  trajectory_controller_state.error.velocities.resize(m);
  trajectory_controller_state.error.effort.resize(m);

  position_initialized = false;

  enabled = false;

  ROS_DEBUG_STREAM("JointPosition::JointPosition: contructor finished");
}

JointPosition::~JointPosition() {
  sub_controller_state_.shutdown();
  watchdog_timer_.stop();
}

void JointPosition::watchdog(const ros::TimerEvent &e) {
  ros::Time now = ros::Time::now();

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_) {
    bool should_abort = false;
    if (!last_controller_state_) {
      should_abort = true;
      ROS_WARN("Aborting goal because we have never heard a controller state "
               "message.");
    } else if ((now - last_controller_state_->header.stamp) >
               ros::Duration(5.0)) {
      should_abort = true;
      ROS_WARN("Aborting goal because we haven't heard from the controller in "
               "%.3lf seconds",
               (now - last_controller_state_->header.stamp).toSec());
    }

    if (should_abort) {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;

      trajectory_index = 0;

      // Marks the current goal as aborted.
      active_goal_.setAborted();
      has_active_goal_ = false;
    }
  }
}

void JointPosition::goalCB(GoalHandle gh) {

  ROS_INFO("JointPosition::goalCB: goal received");

  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names)) {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  // Cancels the currently active goal.
  if (has_active_goal_) {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;

    current_traj_ = empty;
    
    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }

  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;

  // Sends the trajectory along to the controller
  current_traj_ = active_goal_.getGoal()->trajectory;

  trajectory_start = ros::Time::now() + current_traj_.points[0].time_from_start;

  trajectory_index = 0;
}

void JointPosition::cancelCB(GoalHandle gh) {

  ROS_INFO("JointPosition::cancelCB: cancellation request received");

  if (active_goal_ == gh) {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    
    current_traj_ = empty;
    
    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
    trajectory_index = 0;

    traj_length = 0;
  }
}

void JointPosition::controllerStateCB(
    const control_msgs::JointTrajectoryControllerStateConstPtr &msg) {
  last_controller_state_ = msg;
  ros::Time now = ros::Time::now();

  if (!has_active_goal_)
    return;
  if (current_traj_.points.empty())
    return;
  if (now < trajectory_start + current_traj_.points[0].time_from_start)
    return;

  if (!setsEqual(joint_names_, msg->joint_names)) {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  int last = current_traj_.points.size() - 1;
  ros::Time end_time =
      trajectory_start + current_traj_.points[last].time_from_start;

  // Verifies that the controller has stayed within the trajectory constraints.

  if (now < end_time) {
    // Checks that the controller is inside the trajectory constraints.
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      double abs_error = fabs(msg->error.positions[i]);
      double constraint = trajectory_constraints_[msg->joint_names[i]];
      if (constraint >= 0 && abs_error > constraint) {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;

        active_goal_.setAborted();
        has_active_goal_ = false;
        trajectory_index = 0;
        ROS_WARN(
            "Aborting because we would up outside the trajectory constraints");
        return;
      }
    }
  } else {
    // Checks that we have ended inside the goal constraints
    bool inside_goal_constraints = true;
    for (size_t i = 0; i < msg->joint_names.size() && inside_goal_constraints;
         ++i) {
      double abs_error = fabs(msg->error.positions[i]);
      double goal_constraint = goal_constraints_[msg->joint_names[i]];
      if (goal_constraint >= 0 && abs_error > goal_constraint)
        inside_goal_constraints = false;

      // It's important to be stopped if that's desired.
      if (fabs(msg->desired.velocities[i]) < 1e-6) {
        if (fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
          inside_goal_constraints = false;
      }
    }

    if (inside_goal_constraints) {
      active_goal_.setSucceeded();
      has_active_goal_ = false;
      trajectory_index = 0;
    } else if (now < end_time + ros::Duration(goal_time_constraint_)) {
      // Still have some time left to make it.
    } else {
      ROS_WARN("Aborting because we wound up outside the goal constraints");
      active_goal_.setAborted();
      has_active_goal_ = false;
      trajectory_index = 0;
    }
  }
}

void JointPosition::JS_goalCB(JS_GoalHandle gh) {

  ROS_INFO("JointPosition::JS_goalCB: goal received");

  // Ensures that the joints in the goal match the joints we are commanding.
  if (gh.getGoal()->trajectory_name.length() == 0) {
    ROS_ERROR("JointPosition::JS_goalCB: No trajectory points specified");
    gh.setRejected();
    return;
  }

  gh.setAccepted();
  JS_active_goal_ = gh;
  JS_has_active_goal_ = true;

  if (!_nodehandle.hasParam(JS_active_goal_.getGoal()->trajectory_name)) {
    ROS_INFO_STREAM("JointPosition::JS_goalCB: trajectory "
                    << JS_active_goal_.getGoal()->trajectory_name
                    << " does not exist in parameter server");
    return;
  } else {

    XmlRpc::XmlRpcValue waypoints_vector;

    _nodehandle.getParam(JS_active_goal_.getGoal()->trajectory_name,
                         waypoints_vector);

    if (waypoints_vector.getType() == XmlRpc::XmlRpcValue::Type::TypeArray &&
        waypoints_vector.size() > 0) {
      ROS_INFO_STREAM("JointPosition::JS_goalCB: trajectory "
                      << JS_active_goal_.getGoal()->trajectory_name
                      << " loaded with " << waypoints_vector.size()
                      << " points");
      // waypoints_vector[0] is a 'TypeArray' aka vector
    } else {
      ROS_INFO("JointPosition::JS_goalCB: trajectory is empty");
      return;
    }

    boost::shared_ptr<std_msgs::Float64MultiArray> trajectory_point(
        new std_msgs::Float64MultiArray());

    trajectory_point->layout.dim.push_back(
        std_msgs::MultiArrayDimension()); // one structure MultiArrayDimension
                                          // per dimension
    trajectory_point->layout.data_offset = 0;
    trajectory_point->layout.dim[0].label = "";
    trajectory_point->layout.dim[0].size = m + 1;
    trajectory_point->layout.dim[0].stride = 0;

    std::vector<std::string> fields;
    
    for (int i = 0; i < m; i++) fields.push_back("joint_" + std::to_string(i+1));
    
    fields.push_back("time");

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
                "JointPosition::JS_goalCB: trajectory waypoint[%d] has no member %s",
                i, fields[j].c_str());
            return;
          }
        }
      } else {
        ROS_INFO(
            "JointPosition::JS_goalCB: trajectory waypoint[%d] type is not a Struct",
            i);
        return;
      }

      callbackJointSetpoint(trajectory_point);
    }

    trajectory_start = ros::Time::now();

    trajectory_index = 0;
    
    ROS_INFO("JointPosition::JS_goalCB: trajectory is sent");    
  }
}


void JointPosition::JS_cancelCB(JS_GoalHandle gh) {

  ROS_INFO("JointPosition::JS_cancelCB: cancellation request received");

  // Cancels the currently active goal.
  if (JS_has_active_goal_) {
    // clear the trajectory.
    //clear_traj_points();
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    
    current_traj_ = empty;
    
    traj_length = 0;
    
    // Marks the current goal as canceled.
    result_.error_code = result_.INVALID_GOAL;
    JS_active_goal_.setCanceled(result_);
    JS_has_active_goal_ = false;
  }
}

void JointPosition::updateConstraints() {
  ros::Time now = ros::Time::now();
  ros::Time end_time = now;

  trajectory_msgs::JointTrajectoryPoint tmp_point;
  
  if(!enabled) current_traj_.points.clear();

  int last = current_traj_.points.size() - 1;

  trajectory_controller_state.joint_names = joint_names_;

  for (int i = 0; i < m; ++i) {
    x(i, 0) = q(i, 0);
  }

  if (!position_initialized) {
    for (int i = 0; i < m; ++i) {
      xd(i, 0) = q(i, 0);
    }
    if (initialized)
      position_initialized = true;
  }

  if (last > 0)
    end_time = trajectory_start + current_traj_.points[last].time_from_start;

  // Verifies that the controller has stayed within the trajectory constraints.

  trajectory_controller_state.header.stamp = now;

  if (now < end_time) {

   // if (has_active_goal_ || JS_has_active_goal_) {
      for (int i = trajectory_index; i <= last; ++i) {
        /* code */
        if (now <= trajectory_start + current_traj_.points[i].time_from_start) {
          trajectory_index = i;
          break;
        }
      }
    //}

    tmp_point = current_traj_.points[trajectory_index];

  } else {
    if (last > 0)
      tmp_point = current_traj_.points[last];
  }

  for (int i = 0; i < joint_state_.position.size(); i++) {
    if (last > 0)
      for (int j = 0; j < m; ++j) {
        if (current_traj_.joint_names[i] == joint_names_[j]) {
          trajectory_controller_state.desired.positions[j] =
              tmp_point.positions[i];
          // if (now < end_time)
          trajectory_controller_state.desired.velocities[j] =
              tmp_point.velocities[i];
          // else
          // trajectory_controller_state.desired.velocities[j] = 0.0;

          xd(j) = tmp_point.positions[i];
          dxd(j) = tmp_point.velocities[i];
          j = m;
        }
      }
    else 
      for (int j = 0; j < m; ++j) {
        xd(j) = x(j);
        dxd(j) = 0;        
      }

    for (int j = 0; j < m; ++j) {
      if (joint_state_.name[i] == joint_names_[j]) {
        trajectory_controller_state.actual.positions[j] =
            joint_state_.position[i];
        trajectory_controller_state.actual.velocities[j] =
            joint_state_.velocity[i];
        trajectory_controller_state.actual.effort[j] = joint_state_.effort[i];

        j = m;
      }
    }

    trajectory_controller_state.error.positions[i] =
        trajectory_controller_state.desired.positions[i] -
        trajectory_controller_state.actual.positions[i];
    
    q_err(i,0) = trajectory_controller_state.error.positions[i];
        
    trajectory_controller_state.error.velocities[i] =
        trajectory_controller_state.desired.velocities[i] -
        trajectory_controller_state.actual.velocities[i];
    trajectory_controller_state.error.effort[i] =
        trajectory_controller_state.desired.effort[i] -
        trajectory_controller_state.actual.effort[i];
  }

  pub_controller_state.publish(trajectory_controller_state);
  
  if (JS_has_active_goal_) {
    feedback_.percent_complete =
        ((double)trajectory_index / (double)traj_length) * 100;
    JS_active_goal_.publishFeedback(feedback_);

    if (trajectory_index == traj_length - 1) {

      if (q_err.squaredNorm() > JS_active_goal_.getGoal()->ee_error_th) {
        result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
        JS_active_goal_.setCanceled(result_);
        JS_has_active_goal_ = false;
      } else {
        result_.error_code = result_.SUCCESSFUL;
        JS_active_goal_.setSucceeded(result_);
        JS_has_active_goal_ = false;
      }
    }
  }
}

void JointPosition::callbackJointSetpoint(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
    
  Eigen::Matrix<long double, Eigen::Dynamic, 1> traj_point;
  
  traj_point.resize(m, 1);
  
  Eigen::Matrix<long double, Eigen::Dynamic, 1> init_point;
  
  init_point.resize(m, 1);
  
  Eigen::Matrix<long double, Eigen::Dynamic, 1> tmp_xd;
  
  tmp_xd.resize(m + 1, 1);
  
  trajectory_msgs::JointTrajectoryPoint tmp_point;

  ros::Duration start;
  
  double cycleHz;

  _nodehandle.getParam("cycleHz", cycleHz);

  if (msg->data.size() < m + 1) {
    ROS_INFO_STREAM("JointPosition::callbackjointgroupsetpoint: input message must contain at least m + 1 elements, last element is time");
    return;
  }
  
  if (msg->data[m] <= 0.0) {
    ROS_INFO_STREAM("JointPosition::callbackjointgroupsetpoint: the m + 1 element is time and must be greater than 0.0");
    return;
  }

  int last = current_traj_.points.size() - 1;
  
  if (!current_traj_.points.empty()) {
    for (int j = 0; j < m; j++) init_point(j, 0) = current_traj_.points.back().positions[j];
    start = current_traj_.points.back().time_from_start;
  } else {
    init_point = x;
    start = ros::Duration(0);
  }

  for (int i = 0; i < m + 1; i++) {
    tmp_xd(i, 0) = msg->data[i];
    // ROS_INFO_STREAM("new setpoint " << i << " " << tmp_xd(i,0));
  }  
  
  tmp_point.positions.resize(m);
  tmp_point.velocities.resize(m);
  tmp_point.accelerations.resize(m);
  tmp_point.effort.resize(m);
    
  for (int i = 0; i < tmp_xd(m, 0) * cycleHz; i++) {

    for (int j = 0; j < m; j++)
    {
        tmp_point.positions[j] = init_point(j, 0) + (tmp_xd(j, 0) - init_point(j, 0)) * (i / (tmp_xd(m, 0) * cycleHz));
        tmp_point.velocities[j] = (tmp_xd(j, 0) - init_point(j, 0)) / tmp_xd(m, 0);
    }
    tmp_point.time_from_start = start + ros::Duration(i / cycleHz);

    current_traj_.points.push_back(tmp_point);
  }

  // set velocity to zero at the last point
  for (int j = 0; j < m; j++){
    tmp_point.positions[j] = tmp_xd(j, 0);
    tmp_point.velocities[j] = 0.0;
  }

  tmp_point.time_from_start = start + ros::Duration(tmp_xd(m, 0));
  
  current_traj_.joint_names = joint_names_;
  
  current_traj_.points.push_back(tmp_point);
  
  traj_length = current_traj_.points.size();
  }

#include <move_rt/controlmanager.hpp>

ControlManager::ControlManager(ros::NodeHandle &nodehandle)
    : _nodehandle(nodehandle) {

  level_manager.clear();

  std::vector<double> save;
  int nb_tasks;

  string param = "";
  _nodehandle.getParam("n", n);
  save.clear();

  q.resize(n, 1);
  q.setZero();

  dq.resize(n, 1);

  dq.setZero();

  dqd.resize(n, 1);

  dqdmax.resize(n, 1);
  dqdmin.resize(n, 1);

  dqd_old.resize(n, 1);
  dqd_old.setZero();

  Q.resize(n, n);

  M.resize(n, n);
  M.setIdentity();

  I.resize(n, n);
  I.setIdentity();

  Qk.resize(n, n);
  alpha.resize(n);

  JK.resize(6, n);
  JK.setZero();

  _nodehandle.getParam("cycleHz", cycleHz);

  _nodehandle.getParam("ordered_joints", ordered_joints);

  ROS_INFO_STREAM("ControlManager::ControlManager: ordered joint list size "
                  << ordered_joints.size());
  for (int i = 0; i < ordered_joints.size(); ++i) {
    ROS_INFO_STREAM("ControlManager::ControlManager: ordered joint "
                    << i << " = " << ordered_joints[i]);
  }

  save.clear();

  _nodehandle.getParam("dqmax", save);

  dqdmax = convert_vector_matrix(save);
  save.clear();

  _nodehandle.getParam("dqmin", save);
  dqdmin = convert_vector_matrix(save);
  save.clear();

  _nodehandle.getParam("eta", eta);
  save.clear();

  _nodehandle.getParam("controller_topic", controller_topic);
  _nodehandle.getParam("joint_state_topic", joint_state_topic);

  std::vector<string> tasks;
  _nodehandle.getParam("TasksHierarchy", tasks);

  nb_tasks = tasks.size();
  ROS_INFO_STREAM("ControlManager::ControlManager: nb_tasks " << nb_tasks);
  
  for (int i = 0; i < tasks.size(); i++) {
    if (tasks[i] == "EmergencyStop")
      level_manager.push_back(new EmergencyStop(nodehandle, q));
    
    if (tasks[i] == "RobotSafetyStop")
      level_manager.push_back(new RobotSafety(nodehandle, dq));

    if (tasks[i] == "JointLimiter")
      level_manager.push_back(new JointLimiter(nodehandle, q));

    if (tasks[i] == "EePosition")
      level_manager.push_back(new EePosition(nodehandle, JK));
    
    if (tasks[i] == "ElbowPosition")
      level_manager.push_back(new ElbowPosition(nodehandle));

    if (tasks[i] == "JointPosition")
      level_manager.push_back(new JointPosition(nodehandle, q, joint_state));
    
    if (tasks[i] == "singularityavoidance")
      level_manager.push_back(new singularityavoidance(nodehandle, JK));
    
    if (tasks[i] == "dqMinimizer")
      level_manager.push_back(new dqMinimizer(nodehandle));

    ROS_INFO_STREAM("ControlManager::ControlManager: "
                    << level_manager[i]->get_task() << " added");
  }

  initializeSubscribers(); // package up the messy work of creating subscribers;
                           // do this overhead in constructor
  initializePublishers();

  trajMsg.layout.dim.push_back(
      std_msgs::MultiArrayDimension()); // one structure MultiArrayDimension per
                                        // dimension
  trajMsg.layout.data_offset = 0;
  trajMsg.layout.dim[0].label = "joint";
  trajMsg.layout.dim[0].size = n;
  trajMsg.layout.dim[0].stride = 0;

  initialized = false;

  ros::Rate rate(cycleHz);

  while (_nodehandle.ok()) {

    update();

    ros::spinOnce();

    rate.sleep();
  }
}

ControlManager::~ControlManager() {}

void ControlManager::update() {

  dqd.setZero();

  Q.setIdentity();

  dqdmin_tmp = dqdmin;
  dqdmax_tmp = dqdmax;

  for (int i = 0; i < level_manager.size(); i++) {
    level_manager[i]->update(initialized);

    update_prioritylevel(level_manager[i]);
  }

  velocitysaturation();

  dqd_old = dqd;

  trajMsg.data.clear();

  for (int i = 0; i < n; i++) {
      if (initialized) trajMsg.data.push_back(dqd(i));
      else trajMsg.data.push_back(0);
  }

  pub_velocity_joints.publish(trajMsg);
}

void ControlManager::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers");
  sub_jacobian_states = _nodehandle.subscribe<sensor_msgs::JointState>(
      joint_state_topic, 1, &ControlManager::callbackJState, this);
}

void ControlManager::initializePublishers() {
  ROS_INFO("Initializing Publishers");
  pub_velocity_joints =
      _nodehandle.advertise<std_msgs::Float64MultiArray>(controller_topic + "/command", 1);
}

void ControlManager::callbackJState(
    const sensor_msgs::JointState::ConstPtr &msg) {

  joint_state = *msg;

  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < msg->position.size(); i++) {
      if (msg->name[i] == ordered_joints[j]) {
        ROS_DEBUG_STREAM("ControlManager::callbackJState: joint_states.name["
                         << i << "] = " << msg->name[i]);
        ROS_DEBUG_STREAM(
            "ControlManager::callbackJState: joint_states.position["
            << i << "] = " << msg->position[i]);

        q(j, 0) = msg->position[i];
        dq(j, 0) = msg->velocity[i];
        
        i = msg->position.size();
      }
    }
  }

  for (int j = 0; j < n; ++j)
    ROS_DEBUG_STREAM("ControlManager::callbackJState: q(" << j
                                                          << ") = " << q(j, 0));
    
  if (!initialized)
    initialized = true;
}

void ControlManager::update_prioritylevel(PriorityLevel *task) {
  JkQk_1 = task->get_JK() * Q;

  wnpJkQk_1Q = f.weighted_normalized_pinv(JkQk_1, task->A, Q, eta);

  wnpJkQk_1I = f.weighted_normalized_pinv(JkQk_1, task->A, I, eta);

  Wk = JkQk_1 * wnpJkQk_1Q;

  Qk = Q * (I - wnpJkQk_1I * JkQk_1);

  Tk = (I - Q * (wnpJkQk_1I)*Wk * task->get_JK());

  dqd = Tk * dqd + Q * wnpJkQk_1I * Wk * task->dx;

  Q = Qk;
}

void ControlManager::velocitysaturation() {
  delta_dqd = dqd;

  alpha.setOnes();

  for (int i = 0; i < n; i++) {
    if (delta_dqd(i, 0) > dqdmax_tmp(i, 0)) {
      alpha(i, 0) = dqdmax_tmp(i, 0) / delta_dqd(i, 0);
      continue;
    }

    if (delta_dqd(i, 0) < dqdmin_tmp(i, 0)) {
      alpha(i, 0) = dqdmin_tmp(i, 0) / delta_dqd(i, 0);
      continue;
    }
  }

  a = alpha.minCoeff();
  if (isnan(a))
    a = 0.0;

  dqd = a * dqd;
}

void ControlManager::show() {
  ROS_INFO_STREAM("n = " << n);

  ROS_INFO_STREAM("eta = " << eta);

  ROS_INFO_STREAM("q = " << q);

  ROS_INFO_STREAM("dqd = " << dqd);
  ROS_INFO_STREAM("dqd_old = " << dqd_old);

  ROS_INFO_STREAM("dqdmax = " << dqdmax);
  ROS_INFO_STREAM("dqdmin = " << dqdmin);

  ROS_INFO_STREAM("Q = " << Q);

  ROS_INFO_STREAM("M = " << M);
}

#include <move_rt/prioritylevel.hpp>

PriorityLevel::PriorityLevel(ros::NodeHandle &nodehandle)
    : _nodehandle(nodehandle), initialized(false) {}

PriorityLevel::~PriorityLevel() {}

void PriorityLevel::update(bool controllerManager_initialized) {

  initialized = controllerManager_initialized;

  updateJk();

  updateConstraints();

  updateA();

  control();
}

void PriorityLevel::update_task() {
  string name = get_task() + "/setK";
  sub_ee_K_setK =
      _nodehandle.advertiseService(name, &PriorityLevel::serviceSetK, this);

  name = get_task() + "/setxm";
  sub_ee_xm =
      _nodehandle.advertiseService(name, &PriorityLevel::serviceSetxm, this);

  name = get_task() + "/setA";
  sub_ee_A =
      _nodehandle.advertiseService(name, &PriorityLevel::serviceSetA, this);

  name = get_task() + "/getA";
  sub_get_A =
      _nodehandle.advertiseService(name, &PriorityLevel::serviceGetA, this);

  name = get_task() + "/setb";
  sub_ee_b =
      _nodehandle.advertiseService(name, &PriorityLevel::serviceSetb, this);

  name = get_task() + "/setEnable";
  sub_set_enable = _nodehandle.advertiseService(
      name, &PriorityLevel::serviceSetEnable, this);

  std::vector<double> save;
  string param = get_task();

  ROS_DEBUG_STREAM("param " << param);
  _nodehandle.getParam(param + "/m", m);
  save.clear();

  _nodehandle.getParam(param + "/K", save);
  K = convert_vector_to_diag_matrix(save, m);

  int n;

  _nodehandle.getParam("n", n);

  x.resize(m);
  x.setZero();

  xd.resize(m);
  xd.setZero();

  dxd.resize(m);
  dxd.setZero();

  dx.resize(m);
  dx.setZero();

  JK.resize(m, n);
  JK.setZero();

  A.resize(m, m);
}

void PriorityLevel::control() { dx = K * (xd - x) + dxd; }

void PriorityLevel::show() {
  ROS_INFO_STREAM("m: " << m << ", ");
  ROS_INFO_STREAM("xm: [");
  for (int i = 0; i < xm.size(); i++)
    if (i < xm.size() - 1)
      ROS_INFO_STREAM(xm(i, 0) << ", ");
    else {
      ROS_INFO_STREAM(xm(i, 0) << "], ");
    }
  ROS_INFO_STREAM("b: [");
  for (int i = 0; i < b.size(); i++)
    if (i < b.size() - 1)
      ROS_INFO_STREAM(b(i, 0) << ", ");
    else

    {
      ROS_INFO_STREAM(b(i, 0) << "], ");
    }

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

  ROS_INFO_STREAM("J: [");
  for (int i = 0; i < JK.rows(); i++)
    for (int j = 0; j < JK.cols(); j++) {
      if (i == JK.rows() - 1 && j == JK.cols() - 1)
        ROS_INFO_STREAM(JK(i, j) << "], ");
      else {
        ROS_INFO_STREAM(JK(i, j) << ", ");
      }
    }

  ROS_INFO_STREAM("A: [");
  for (int i = 0; i < A.rows(); i++)
    for (int j = 0; j < A.cols(); j++) {
      if (i == A.rows() - 1 && j == A.cols() - 1)

        ROS_INFO_STREAM(A(i, j) << "], ");
      else {
        ROS_INFO_STREAM(A(i, j) << ", ");
      }
    }
  ROS_INFO_STREAM("K: [");
  for (int i = 0; i < K.rows(); i++)
    for (int j = 0; j < K.cols(); j++) {
      if (i == K.rows() - 1 && j == K.cols() - 1)
        ROS_INFO_STREAM(K(i, j) << "]} ");
      else {
        ROS_INFO_STREAM(K(i, j) << ", ");
      }
    }
}

bool PriorityLevel::serviceSetK(move_rt::TaskParamUpdate::Request &req,
                                move_rt::TaskParamUpdate::Response &res) {
  if (req.data.size() != m) {
    ROS_INFO_STREAM(get_task() << "::serviceSetK: incorrect request size\n");
    res.result = false;
    return false;
  }

  for (int j = 0; j < m; j++) {
    K(j, j) = req.data[j];
  }

  ROS_INFO_STREAM(get_task() << "::serviceSetK: K = \n" << K);

  res.result = true;
  return true;
}

bool PriorityLevel::serviceSetxm(move_rt::TaskParamUpdate::Request &req,
                                 move_rt::TaskParamUpdate::Response &res) {
  if (req.data.size() != m) {
    ROS_INFO_STREAM(get_task() << "::serviceSetxm: incorrect request size\n");

    res.result = false;
    return false;
  }

  for (int j = 0; j < m; j++) {
    xm(j, 0) = req.data[j];
  }

  ROS_INFO_STREAM(get_task() << "::serviceSetxm: xm = \n" << xm);

  res.result = true;
  return true;
}

bool PriorityLevel::serviceSetb(move_rt::TaskParamUpdate::Request &req,
                                move_rt::TaskParamUpdate::Response &res) {

  if (req.data.size() != m) {
    ROS_INFO_STREAM(get_task() << "::serviceSetb: incorrect request size\n");

    res.result = false;
    return false;
  }

  for (int j = 0; j < m; j++) {
    b(j, 0) = req.data[j];
  }

  ROS_INFO_STREAM(get_task() << "::serviceSetb: b = \n" << b);

  res.result = true;
  return true;
}

bool PriorityLevel::serviceSetA(move_rt::TaskParamUpdate::Request &req,
                                move_rt::TaskParamUpdate::Response &res) {
  if (req.data.size() != m) {
    ROS_INFO_STREAM(get_task() << "::serviceSetA: incorrect request size\n");

    res.result = false;
    return false;
  }

  for (int j = 0; j < m; j++) {
    A(j, j) = req.data[j];
  }

  ROS_INFO_STREAM(get_task() << "::serviceSetA: A = \n" << A);

  res.result = true;
  return true;
}

bool PriorityLevel::serviceGetA(move_rt::TaskParamUpdate::Request &req,
                                move_rt::TaskParamUpdate::Response &res) {
  ROS_INFO_STREAM(get_task() << "::serviceGetA: A = \n" << A);

  res.result = true;
  return true;
}

bool PriorityLevel::serviceSetEnable(move_rt::TaskParamUpdate::Request &req,
                                     move_rt::TaskParamUpdate::Response &res) {
  if (req.data.size() != 1) {
    ROS_INFO_STREAM(get_task()
                    << "::serviceSetEnable: incorrect request size\n");
    res.result = false;
    return false;
  }

  if (req.data[0] == 1) {
    
    xd = x;
    
    A.setIdentity();
    enabled = true;
    ROS_INFO_STREAM(get_task() << "::serviceSetEnable: enabled\n");
  } else if (req.data[0] == 0) {
    A.setZero();
    enabled = false;
    ROS_INFO_STREAM(get_task() << "::serviceSetEnable: disabled\n");
  } else {
    ROS_INFO_STREAM(get_task() << "::serviceSetEnable: incorrect request "
                                  "value, use 0 to disable, 1 to enable\n");
    res.result = false;
    return false;
  }

  res.result = true;
  return true;
}

#include <move_rt/jointlimiter.hpp>

JointLimiter::JointLimiter(ros::NodeHandle &nodehandle,
                           Eigen::Matrix<long double, Eigen::Dynamic, 1> &_q)
    : PriorityLevel(nodehandle), q(_q) {
  ROS_DEBUG_STREAM("enter JointLimiter contructor ");

  PriorityLevel::update_task();

  std::vector<double> save;
  string param = get_task();

  for (int i = 0; i < m / 2; i++) {
    JK(2 * i, i) = 1;
    JK(2 * i + 1, i) = 1;
  }

  nodehandle.getParam(param + "/xm", save);
  xm = convert_vector_matrix(save);
  save.clear();

  nodehandle.getParam(param + "/b", save);
  b = convert_vector_matrix(save);
  save.clear();
}

JointLimiter::~JointLimiter() {}

void JointLimiter::updateA() {
  A.setZero();

  for (int i = 0; i < m; i++) {

    if (i % 2 == 0) {      
        A(i, i) = 1 - sigmoid(x(i), xm(i) - b(i), b(i));    
    } else {
        A(i, i) = sigmoid(x(i), xm(i), b(i));
    }
  }
  ROS_DEBUG_STREAM("A \n" << A);
}

void JointLimiter::updateConstraints() {
  for (int i = 0; i < m / 2; i++) {
    xd(2 * i, 0) = xm(2 * i) - b(2 * i);
    xd(2 * i + 1, 0) = xm(2 * i + 1) + b(2 * i + 1);
    x(2 * i, 0) = q(i, 0);      //- xd(2*i,0);
    x(2 * i + 1, 0) = q(i, 0); // + xd(2*i,0);
  }
  ROS_DEBUG_STREAM("x \n" << x);
  ROS_DEBUG_STREAM("xd \n" << xd);
}

string JointLimiter::get_task() { return "JointLimiter"; }

void JointLimiter::show() {
  PriorityLevel::show();
}

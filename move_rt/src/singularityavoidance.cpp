#include <move_rt/singularityavoidance.hpp>

singularityavoidance::singularityavoidance(
    ros::NodeHandle &nodehandle,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> &_JK)
    : PriorityLevel(nodehandle), JK_manipulator(_JK) {
  ROS_DEBUG_STREAM("enter singularityavoidance contructor ");

  PriorityLevel::update_task();

  std::vector<double> save;
  string param = get_task();

  nodehandle.getParam("n", n);

  d.resize(6, n);

  nodehandle.getParam(param + "/xm", save);
  xm = convert_vector_matrix(save);
  save.clear();

  nodehandle.getParam(param + "/b", save);
  b = convert_vector_matrix(save);
  save.clear();

  ROS_DEBUG_STREAM("singularityavoidance contructor finished");
}

singularityavoidance::~singularityavoidance() {}

void singularityavoidance::updateJk() {
  for (int i = 0; i < JK.rows(); i++)
    for (int j = 0; j < JK.cols(); j++)
      d(i, j) = JK_manipulator(i, j);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(d, ComputeFullU | ComputeFullV);

  v = svd.matrixV();
  singularValues = svd.singularValues();

  for (int i = 0; i < v.rows(); i++)
    JK(0, i) = v(i, 5);
}

void singularityavoidance::updateA() { A(0, 0) = sigmoid(x(0), xm(0), b(0)); }

void singularityavoidance::updateConstraints() {
  x(0, 0) = singularValues(n - 1);
  xd(0, 0) = 1.6;
}

string singularityavoidance::get_task() { return "singularityavoidance"; }

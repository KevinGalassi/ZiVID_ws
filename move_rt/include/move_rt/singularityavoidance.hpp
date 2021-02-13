
#ifndef SINGULARITYAVOIDANCE_H
#define SINGULARITYAVOIDANCE_H

#include <move_rt/eeposition.hpp>
#include <move_rt/prioritylevel.hpp>
#include <vector>

class singularityavoidance : public PriorityLevel {
public:
  singularityavoidance(
      ros::NodeHandle &nodehandle,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> &_JK);
  ~singularityavoidance();

  void updateJk();
  void updateA();
  void updateConstraints();

  string get_task();

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };

private:
  int n;
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> &JK_manipulator;

  Eigen::MatrixXd d, v;
  Eigen::VectorXd singularValues;
};

#endif // SPEEDSAFETY_H

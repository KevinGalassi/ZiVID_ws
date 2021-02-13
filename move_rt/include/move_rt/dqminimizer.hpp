#ifndef DQMINIMIZER_H
#define DQMINIMIZER_H

#include <move_rt/prioritylevel.hpp>

/// @brief dqMinimizer is one type of PriorityLevel which is the minimizer of
/// the joint velocities consuming all remaining degrees of freedom. This task
/// is responsible of updating all its task parameters

class dqMinimizer : public PriorityLevel {
public:
  /**
   * Constructor of dqMinimzer.
   *
   * @param nodehandle is a pointer to a ros nodeHandle
   */
  dqMinimizer(ros::NodeHandle &nodehandle);

  /**
   * Destructor of the dqMinimizer.
   */
  ~dqMinimizer();

  /**
   * control: overwrite the default control function with an empty one.
   */
  void control();

  /**
   * get_task: return the name of the actual class.
   */
  string get_task();

  /**
   * get_JK: returns the task Jacobian.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };
};

#endif // DQMINIMIZER_H

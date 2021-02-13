#ifndef JOINTLIMITER_H
#define JOINTLIMITER_H

#include <move_rt/prioritylevel.hpp>

/// @brief JointLimiter is one type of PriorityLevel which is the the
/// JointLimiter controller.
class JointLimiter : public PriorityLevel {
public:
  /**
   * Constructor of JointLimiter.
   *
   * @param nodehandle is a pointer to a ros nodeHandle
   * @param _q is a pointer to the joint positions
   */
  JointLimiter(ros::NodeHandle &nodehandle,
               Eigen::Matrix<long double, Eigen::Dynamic, 1> &_q);

  /**
   * Destructor of the JointLimiter.
   */
  ~JointLimiter();

  // void updatexm(Eigen::Matrix<long double,Eigen::Dynamic,1> xm);

  /**
   * updateA: update the Activation matrix of JointLimiter task.
   */
  void updateA();

  /**
   * updateConstraints: update the x and xd vectors of JointLimiter task.
   */
  void updateConstraints();

  /**
   * show: prints the parameters of the current task controller.
   */
  void show();

  /**
   * get_task: returns the name of the actual class.
   */
  string get_task();

  /**
   * get_JK: returns the task Jacobian.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };

  Eigen::Matrix<long double, Eigen::Dynamic, 1> &q;
};

#endif // JOINTLIMITER_H

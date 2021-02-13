#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H

#include <move_rt/prioritylevel.hpp>

static bool setsEqual(const std::vector<std::string> &a,
                      const std::vector<std::string> &b);

/// @brief EmergencyStop is the robot joint position controller to interface
/// with the Moveit! planners
class EmergencyStop : public PriorityLevel {
public:
  /**
   * Constructor of EmergencyStop.
   *
   * @param nodehandle is a pointer to a ros nodeHandle
   * @param _q is a pointer to the robot joint positions
   * @param joint_state is a pointer to the robot joint state message
   */
  EmergencyStop(ros::NodeHandle &nodehandle,
                Eigen::Matrix<long double, Eigen::Dynamic, 1> &_q);

  /**
   * Destructor of the EmergencyStop.
   */
  ~EmergencyStop();

  /**
   * updateConstraints: update the x and xd vectors of EePosition task.
   */
  void updateConstraints();

  /**
   * get_task: return the name of the actual class.
   */
  string get_task() { return "EmergencyStop"; };

  /**
   * get_JK: returns the task Jacobian.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };

private:
  bool position_initialized;

  Eigen::Matrix<long double, Eigen::Dynamic, 1> &q;
};

#endif // EMERGENCYSTOP_H

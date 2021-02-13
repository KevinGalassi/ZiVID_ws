#ifndef JOINTLIMITER_H
#define JOINTLIMITER_H

#include <ros/ros.h> //ALWAYS need to include this

#include <move_rt/controlmanager.hpp>
#include <move_rt/prioritylevel.hpp>

class JointLimiter : public PriorityLevel {
public:
  JointLimiter(ros::NodeHandle *nodehandle,
               Eigen::Matrix<long double, Eigen::Dynamic, 1> *_q);
  ~JointLimiter();

  // void updatexm(Eigen::Matrix<long double,Eigen::Dynamic,1> xm);
  void updateA();
  void updateConstraints();
  void control();
  void show();

  string get_task();

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> get_JK() {
    return JK;
  };

  Eigen::Matrix<long double, Eigen::Dynamic, 1> qmax, qmin;
  Eigen::Matrix<long double, Eigen::Dynamic, 1> *q;
};

#endif // JOINTLIMITER_H

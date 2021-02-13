#include <move_rt/dqminimizer.hpp>

dqMinimizer::dqMinimizer(ros::NodeHandle &nodehandle)
    : PriorityLevel(nodehandle) {
  PriorityLevel::update_task();

  JK.setIdentity();

  A.setIdentity();
}

dqMinimizer::~dqMinimizer() {}

void dqMinimizer::control() {}

string dqMinimizer::get_task() { return "dqMinimizer"; }

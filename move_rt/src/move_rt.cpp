#include <move_rt/controlmanager.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_rt"); // node name

  ros::NodeHandle
      nh; // create a node handle; need to pass this to the class constructor

  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  // ros::console::levels::Debug);

  ROS_INFO("main: going into spin; let the callbacks do all the work");

  double cycleHz = 100;

  ControlManager cm(nh);

  return 0;
}

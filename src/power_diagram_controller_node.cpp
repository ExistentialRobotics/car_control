#include <ros/ros.h>
#include "car_control/power_diagram_controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  car_control::PowerDiagramController huskyHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}

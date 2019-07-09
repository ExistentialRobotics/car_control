#include <ros/ros.h>
#include "car_control/power_diagram_controller.hpp"

int main(int argc, char** argv)
{
  ROS_INFO("Starting power_diagram_controller");
  ros::init(argc, argv, "power_diagram_controller");
  ros::NodeHandle nodeHandle("~");

  car_control::PowerDiagramController powerDiagramController(nodeHandle);

  ros::spin();
  return 0;
}

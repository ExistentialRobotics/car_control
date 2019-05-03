#include "car_control/power_diagram_controller.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <math.h>

namespace car_control {

PowerDiagramController::PowerDiagramController(ros::NodeHandle& nodeHandle) :
    node_handle(nodeHandle)
{
  if (!this->getParams())
  {
    ROS_ERROR("Missing parameters");
    ros::requestShutdown();
  }

  path = nav_msgs::Path();
  cmd_pub = node_handle.advertise<geometry_msgs::Twist>( cmd_topic, 1 );
  path_pub= node_handle.advertise<nav_msgs::Path>( path_topic, 1 );
  odom_sub= node_handle.subscribe(odom_topic,1,&PowerDiagramController::odomCallback,this);
}

PowerDiagramController::~PowerDiagramController()
{
}

bool PowerDiagramController::getParams()
{
  if (!node_handle.getParam("/power_diagram_controller/cmd_vel",cmd_topic))
    return false;
  if (!node_handle.getParam("/power_diagram_controller/path", path_topic))
    return false;
  if (!node_handle.getParam("/power_diagram_controller/odom", odom_topic))
    return false;
  if (!node_handle.getParam("/power_diagram_controller/kl", kl))
    return false;
  if (!node_handle.getParam("/power_diagram_controller/ka", ka))
    return false;
  if (!node_handle.getParam("/power_diagram_controller/goal_threshold", goal_threshold))
    goal_threshold = 0.1;
  if (!node_handle.getParam("/power_diagram_controller/T", T))
    T = 10;
}
void PowerDiagramController::odomCallback(const nav_msgs::Odometry& odoms)
{
  // Publish path
  PowerDiagramController::pubPath(odoms);

  // Calculate x, y, and yaw from odom
  odom_x = odoms.pose.pose.position.x;
  odom_y = odoms.pose.pose.position.y;
  odom_yaw = tf::getYaw(odoms.pose.pose.orientation);
  //ROS_INFO("The car location is [%.3f,%.3f],%.3f",odom_x,odom_y,odom_yaw);

  // Get Twist commands
  geometry_msgs::Twist cmd_vel = this->getCmd();

  // Publish cmd_vel
  cmd_pub.publish(cmd_vel);
}

geometry_msgs::Twist PowerDiagramController::getCmd()
{
  ros::Time current_t = ros::Time::now();
  ros::Duration delta_t = current_t - init_t;
  // Normalized time
  double t = std::min(delta_t.toSec(), T)/T;
  double goal_x, goal_y;
  // Current goal to follow at time t
  this->calculatePath(t, init_x, init_y, goal_x, goal_y);
  double linear_vel;
  double angular_vel;

  if ((fabs(odom_x - goal_x) < goal_threshold) && (fabs(odom_y - goal_y) < goal_threshold))
  {
    // Reached goal! Stop here!
    linear_vel = 0;
    angular_vel = 0;
  }
  else
  {
    // power diagram control
    linear_vel = -kl*(cos(odom_yaw)*(odom_x - goal_x) + sin(odom_yaw)*(odom_y - goal_y));
    angular_vel = -ka*atan2(-sin(odom_yaw)*(odom_x - goal_x) + cos(odom_yaw)*(odom_y - goal_y), cos(odom_yaw)*(odom_x - goal_x) + sin(odom_yaw)*(odom_y - goal_y));
  }

  geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;
  return cmd_vel;
}

void PowerDiagramController::pubPath(const nav_msgs::Odometry& odoms)
{
  // Only recalculate the path if we want to update. Otherwise, publish the current path.
    if (update_path)
    {
      update_path = false;
      init_x = odoms.pose.pose.position.x;
      init_y = odoms.pose.pose.position.y;
      init_t = ros::Time::now();
      path = nav_msgs::Path();
      path.header.stamp = init_t;
      path.header.frame_id = odoms.header.frame_id;
      this->genPath(1000);
    }

    // Publish path
    path_pub.publish(path);
}

void PowerDiagramController::genPath(int point_count)
{
  double xt;
  double yt;
  for(int i = 0; i<= point_count; i++)
  {
    geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
    pose.header.frame_id = path.header.frame_id;
    pose.header.stamp = ros::Time::now();
    double t = double(i)/point_count;
    this->calculatePath(t, init_x, init_y, xt, yt);
    pose.pose.position.x = xt;
    pose.pose.position.y = yt;
    path.poses.push_back(pose);
  }
}

void PowerDiagramController::calculatePath(double t, double x0, double y0, double &xt, double &yt)
{
  double vx = 1.0, vy = 5.0;
  double ax = -10.0, ay = 0.2;
  xt = x0 + vx*t + ax*t*t/2;
  yt = y0 + vy*t + ay*t*t/2;
}

} /* namespace */

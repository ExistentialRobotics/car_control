#pragma once

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"


namespace car_control {

/*!
 * Class containing the Power Diagram Controller
 */
class PowerDiagramController {
public:
	/*!
	 * Constructor.
	 */
  PowerDiagramController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~PowerDiagramController();
  /*!
   * Get params from launch file.
   */
	bool getParams();
  /*!
   * Odometry callback
   */
	void odomCallback(const nav_msgs::Odometry& odoms);
  /*!
   * calculate the position on the path at normalized time t, 0<=t<=1.
   */
	void calculatePath(double t, double x0, double y0, double &xt, double &yt);
  /*!
   * Generate path
   */
	void genPath(int point_count);
  /*!
   * Publish path
   */
	void pubPath(const nav_msgs::Odometry& odoms);
  /*!
   * Calculate twist according to power diagram controller
   */
	geometry_msgs::Twist getCmd();

private:
	// Publishers and Subscribers
	ros::NodeHandle node_handle;
	ros::Publisher cmd_pub;
	ros::Publisher path_pub;
	ros::Subscriber odom_sub;
	std::string cmd_topic, path_topic, odom_topic;

	// Current robot pose
	double odom_x;
	double odom_y;
	double odom_yaw;
	// Initial robot's position (start) and time.
	double init_x;
	double init_y;
	ros::Time init_t;

	// Path params
	bool update_path = true;
	nav_msgs::Path path;

  // Controller params
  // T is the desired time to go from start to goal
  double T;
	// Controller gains
	double kl, ka, goal_threshold;
	double vx, vy, ax, ay;

};

} /* namespace */

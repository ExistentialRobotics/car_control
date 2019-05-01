#pragma once

#include <ros/ros.h>

namespace car_control {

/*!
 * Class containing the Husky Highlevel Controller
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

private:
	ros::NodeHandle nodeHandle_;
};

} /* namespace */

# Ground robot controller
This repo contains code for car controllers.
## Power Diagram controller
This node calculates twist so that a differential drive robot follows a given polynomial trajectory based on this paper: https://ieeexplore.ieee.org/document/7487090

### Launch script
    roslaunch car_control scarab_control.launch

 	<!-- Run controller in C++ -->
  	<node name="power_diagram_controller" pkg="car_control" type="car_control" output="screen">
    	<param name="/cmd_vel" value="/scarab40/mobile_base/commands/velocity" />
    	<param name="/path" value="/path" />
    	<param name="/odom" value="/scarab40/ground_truth/odom" />
    	<param name="/kl" value="0.5" />
    	<param name="/ka" value="0.1" />
    	<param name="/goal_threshold" value="0.1" />
    	<param name="/T" value="40" />
  	</node>
  
### Examples:

![picture](figures/scarab1.png)

![picture](figures/scarab2.png)

![picture](figures/scarab3.png)

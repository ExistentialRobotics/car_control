# Differential Drive Controller
This node controls a differential drive robot to reach a desired position. 

The controller is based on the intuition from https://cs.gmu.edu/~kosecka/cs485/lec04-control.pdf

## C++ and ROS Usage

Example Launch script:

    <!--Launch a DD Controller Node -->
    <group ns="$(arg ns)">
      <node name="dd_control_node" pkg="dd_control" type="dd_control_node" output="screen"
      args="">
            <remap from="~odom" to="ground_truth/odom"/>
            <remap from="~commands" to="mobile_base/commands/velocity"/>
            <rosparam file="$(find dd_control)/config/dd_gains.yaml"/>
      </node>
    </group>
  

## Python Usage

    cd script
    python3 dd_controller.py

![Main Controller](data/main.png)

![Main Controller](data/error.png)

#!/usr/bin/env python3
""" Low level velocity conttoller for unicycle-like robot. 

Interfaces:
    Input:
            desired robot states (z*) from ref_gvn node z* = zg
            current robot states (z) from odometry 
    Output:
            velocity command for mobile platform or simulated dynamics

"""

from __future__ import print_function

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped


def clip(x, x_min, x_max):
    """
    clip x in [x_min, xmax]
    """
    if x < x_min:
        x = x_min
    if x > x_max:
        x = x_max
    return x


def wrap_angle_pmp(angle_vec):
    """
    npla.normalize angle in radian to [-pi, pi)
    angle_vec: angle despcription in radian
    """
    angle_vec = (angle_vec + np.pi) % (2 * np.pi) - np.pi
    return angle_vec


class UnicycleControlPreprocess:
    """
    Unicycle Control Preprocess Module. Responsible for:
        1) take ros odom message generate robot states z (e.g., unicycle states z = (x, y, theta))
        2) optional state transformation, e.g., nonlinear->linear, cartesian-> polar for low level controller
    """

    # Running status table, higher number better status

    def __init__(self):

        # params from launch file
        _setpoint_topic = rospy.get_param('~setpoint_topic')
        _setpoint_type = rospy.get_param('~setpoint_type', "poseS")
        _odom_topic = rospy.get_param('~odom_topic')
        _odom_type = rospy.get_param('~odom_type', "poseS")

        # subscribers
        if _odom_type == "poseS":
            self._odom_sub = rospy.Subscriber(_odom_topic, PoseStamped, self.odom_callback)
            rospy.logwarn("Odometry message type is specified to: [PoseStamped]")
        else:
            self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback)
            rospy.logwarn("Odometry message type is specified to: [Odometry]")

        if _setpoint_type == "poseS":
            self._setpoint_sub = rospy.Subscriber(_setpoint_topic, PoseStamped, self.setpoint_callback)
            rospy.logwarn("Setpoint message type is specified to: [PoseStamped]")
        else:
            self._setpoint_sub = rospy.Subscriber(_setpoint_topic, Pose2D, self.setpoint_callback)
            rospy.logwarn("Setpoint message type is specified to: [Pose2D]")

        # upstream status variables
        self._upstream_connection = 0
        self._upstream_connection_ready = False
        self._upstream_data_ready = False

        # ------------------- upstream data container --------------------

        # containers for converted message in numpy
        self._np_z = None  # robot states from odom
        self._np_z_dsr = None  # desired robot states from high level controller

        # ------------------- Init Upstream --------------------
        self.init_upstream()
        rospy.loginfo("[Cone Controller Preprocessor Created!]  \n")

    def _check_upstream_connections(self, upstream_connection=2):
        """ check whether subscribers' uplink connections are established """

        self._upstream_connection = \
            self._odom_sub.get_num_connections() + \
            self._setpoint_sub.get_num_connections()

        if self._upstream_connection < upstream_connection:
            # we need to wait states, setpoint ready
            rospy.loginfo('[cone controller] waiting upstream connections [%d / %d]:', self._upstream_connection,
                          upstream_connection)

            # odom
            if self._odom_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[cone controller] waiting odom...")

            # setpoint
            if self._setpoint_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[cone controller] waiting setpoint...")
        else:
            self._upstream_connection_ready = True
            rospy.loginfo("\n[cone controller] %d upstream connections established !\n", upstream_connection)

    def _check_upstream_data(self):
        """ check whether upstream data container are loaded/initialized correctly"""
        status = True

        # robot state z
        if self._np_z is None:
            status = False
            rospy.loginfo_throttle(1.0, "[cone controller] waiting zvec init...")

        # desired robot state z* (setpoint), for ref_gvn high level controller z* = zg
        if self._np_z_dsr is None:
            status = False
            rospy.loginfo_throttle(1.0, "[cone controller] waiting zvec_dsr init...")

        if status:
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[cone controller] all %d upstream data initialized !\n")

    def init_upstream(self):
        """
        Init upstream of unicycle controller.
            1. check upstream connection
            2. check upstream message and initialize downstream data containers
        """
        while (not self._upstream_connection_ready) and (not rospy.is_shutdown()):
            self._check_upstream_connections()
            rospy.sleep(0.1)  # avoid inquery too fast
            rospy.loginfo_throttle(1.0, "[cone controller] waiting upstream connections...")
        rospy.loginfo("upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)  # avoid inquery too fast

        rospy.loginfo_once("[cone controller] upstream [data] is ready!")
        rospy.loginfo("[cone controller] Upstream Init Done!")

    def odom_callback(self, msg_odom):
        rospy.logdebug("[cone controller pre] Received odometry!")
        pose = None

        if isinstance(msg_odom, Odometry):
            pose = msg_odom.pose.pose
        elif isinstance(msg_odom, PoseStamped):
            pose = msg_odom.pose
        else:
            rospy.logwarn("Received unknown odometry message type")

        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
        self._np_z = np.array([pose.position.x, pose.position.y, yaw])

        return

    def setpoint_callback(self, msg_pose):
        rospy.logdebug("Received setpoint!")
        if isinstance(msg_pose, Pose2D):
            self._np_z_dsr = np.array([msg_pose.x, msg_pose.y, msg_pose.theta])
        elif isinstance(msg_pose, PoseStamped):
            quaternion_sxyz = [msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w]
            (_, _, theta) = euler_from_quaternion(quaternion_sxyz)
            self._np_z_dsr = np.array([msg_pose.pose.position.x, msg_pose.pose.position.y, theta])
        else:
            rospy.logwarn("Received unknown setpoint message type")

        return


class UnicycleControlPostprocess:
    """
    Unicycle Controller Post-process Module. Responsible for:
        1) create downstream publisher interface
        2) optional state transformation, e.g., linear->nonlinear, polar->cartesian
        3) clip control with repsect to hardware limits
    """

    def __init__(self, ctrl_limits=None):

        # publishers
        # publish velocity command (body twist) for mobile platform hardware / simulated dynamics
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._body_twist = Twist()
        rospy.loginfo("[cone controller post-processor initialized!]")

        if ctrl_limits is not None:
            self.ctrl_limits = ctrl_limits

    def send_cmd(self, v_dsr, w_dsr, clip_ctrl=False, debug=False):
        """
        Publish low level command velocity to hardware or simulated dynamics
        For unicycle-like robot, desired linear and angular velocity
        """
        if debug:
            rospy.logwarn_throttle(0.5, "Input body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr))

        if clip_ctrl and self.ctrl_limits is not None:
            v_dsr = clip(v_dsr, self.ctrl_limits['v_min'], self.ctrl_limits['v_max'])
            w_dsr = clip(w_dsr, self.ctrl_limits['w_min'], self.ctrl_limits['w_max'])

        if debug:
            rospy.logwarn_throttle(0.5,"Output body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr))

        self._body_twist.linear.x = v_dsr
        self._body_twist.angular.z = w_dsr
        self.cmd_vel_pub.publish(self._body_twist)

        return


class ConeController:
    """
    Cone controller from Omur's technical report. Using this to compute
    velocity control signal given current and desired robot states.
    """

    # Running status table, higher number better status
    NORMAL = 0
    GOAL_LOC_REACHED = 1
    GOAL_POSE_REACHED = 2

    def __init__(self, ctrl_params):
        """
        Init cone controller
        Input:
            @ctrl_params: controller design parameters
        """
        if ctrl_params is not None:
            # controller design parameters
            self.kv = ctrl_params["kv"]
            self.kw = ctrl_params["kw"]
        else:
            print("[ConeController] use default params")
            self.kv = 0.5
            self.kw = 1.5

        self.warning_msg = None
        self.status = ConeController.NORMAL
        self._goal_pose_reached_announced = False

    def generate_control(self, z, z_dsr,
                         eps_dist=0.1,
                         eps_dist_reset=0.3,
                         eps_angle=0.05,
                         eps_angle_reset=0.2,
                         sddm_boost=1.0,
                         debug=False):
        """
        Generate velocity control signal (v, omega) given current robot states and desired robot states.
        Input:
            @z: current robot states (x, y, theta)
            @z_dsr: desired robot states (x*, y*, theta*)
            @eps_dist: goal region tolerance (meter)
            @eps_angle: goal pose angle tolerance in rad
            @eps_dist_reset: hysteresis reset mechanism for dist
            @eps_angle_reset: hysteresis reset mechanism for angle
        Output:
            @cmd_v: raw linear velocity command (m/sec)
            @cmd_w: raw angular velocity command (rad/sec)
        """

        if not self._goal_pose_reached_announced:
            rospy.logdebug_throttle(0.5,
                                    "[cone controller] current status = --------------------------- %s" % self.status)
            if self.status == ConeController.GOAL_POSE_REACHED:
                rospy.logwarn("[cone controller] GOAL POSE REACHED !!")
                self._goal_pose_reached_announced = True

        e = z_dsr[0:2] - z[0:2]  # positional vector from current position to goal position
        err_dist_norm = np.linalg.norm(e)
        err_angle = wrap_angle_pmp(z_dsr[2] - z[2])
        err_angle_norm = np.abs(err_angle)

        msg = None
        new_status = self.status
        # ----------------------- Finite State Machine -----------------------
        # hystersis state jump to combat against noise

        # start with pose_reached status
        if self.status == ConeController.GOAL_POSE_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeController.NORMAL
                msg = "[cone controller] status [down] [pose --> normal] triggered by [dist] err"
                msg += ": dist err > eps_dist_reset (%.3f > %.3f)" % (err_dist_norm, eps_dist_reset)
                rospy.logwarn_throttle(0.5, msg)
                self._goal_pose_reached_announced = False
            # distance error <= eps_dist_reset
            else:
                if err_angle_norm > eps_angle_reset:
                    new_status = ConeController.GOAL_LOC_REACHED
                    msg = "[cone controller] status [down] [pose --> loc] triggered by [angle] err"
                    msg += ": angle err > eps_angle_reset (%.3f > %.3f)" % (err_angle_norm, eps_angle_reset)
                    rospy.logwarn_throttle(0.5, msg)
                    self._goal_pose_reached_announced = False
                # distance error <= eps_dist_reset, angle error <= eps_angle_reset
                else:
                    # remain at POSE_REACHED
                    pass

        # start with loc_reached status
        if self.status == ConeController.GOAL_LOC_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeController.NORMAL
                msg = "[cone controller] status [down] [loc --> normal] triggered by [dist] err"
                msg += ": dist err > eps_dist_reset (%.3f > %.3f)" % (err_dist_norm, eps_dist_reset)
                rospy.logwarn_throttle(0.5, msg)
            # distance error <= eps_dist_reset
            else:
                if err_angle_norm <= eps_angle:
                    new_status = ConeController.GOAL_POSE_REACHED
                    msg = "[cone controller] status [ up ] [loc --> pose]"
                    msg += ": |angle err| <= eps_angle (%.3f < %.3f)" % (err_angle_norm, eps_angle)
                    rospy.logwarn_throttle(0.5, msg)
                # distance error <= eps_dist_reset, angle error > eps_angle
                else:
                    # remain at LOC_REACHED
                    pass

        # start with normal status
        if self.status == ConeController.NORMAL:
            if err_dist_norm <= eps_dist:
                new_status = ConeController.GOAL_LOC_REACHED
                msg = "[cone controller] status [ up ] [normal --> loc]"
                msg += ": dist err <= eps_dist (%.3f > %.3f)" % (err_dist_norm, eps_dist)
                rospy.logwarn_throttle(0.5, msg)
            # distance error > eps_dist
            else:
                # remain at NORMAL
                pass

        # -------------------- applied control strategy by status --------------
        # stay static
        if new_status == ConeController.GOAL_POSE_REACHED:
            cmd_v = 0.0
            cmd_w = 0.0
        # turn in place
        angular_velocity_sf = 0.3  # angular velocity scale factor, applied when close to goal, prevent turn-in-place drifting.
        if new_status == ConeController.GOAL_LOC_REACHED:
            cmd_v = 0.0
            # when close to goal, slow turn, prevent turn-in-place induced position drifting
            cmd_w = angular_velocity_sf * self.kw * err_angle
            rospy.logdebug_throttle(0.5,
                                    "[llc = propotional controller] self.kw = %.2f, err_angle = %.2f, cmd_w = %.2f" % (
                                    self.kw, err_angle, cmd_w))
        # cone controller
        if new_status == ConeController.NORMAL:
            rospy.logdebug_throttle(0.5, "[llc  = cone controller] active")
            # ------------------ normal case  ----------------
            theta = z[2]
            u1 = np.array([np.cos(theta), np.sin(theta)])  # heading direction
            u2 = np.array([-np.sin(theta), np.cos(theta)])  # R_ccw(pi/2) * u1

            e_proj_u1 = np.inner(u1, e)
            e_proj_u2 = np.inner(u2, e)

            cmd_v = sddm_boost * self.kv * max(0, e_proj_u1)

            # when close to goal, the angle error is sensitive due to atan2 discontinuity, apply angular velocity scaling
            if np.abs(e_proj_u1) < 2.0 * eps_dist:
                cmd_w = angular_velocity_sf * self.kw * np.arctan2(e_proj_u2, e_proj_u1)
                rospy.logdebug_throttle(0.5, "[cone controller] close too goal")
            else:
                cmd_w = self.kw * np.arctan2(e_proj_u2, e_proj_u1)

            if debug:
                print("input z = [%.2f, %.2f, %.2f]" % (z[0], z[1], z[2]))
                print("input z_dsr = [%.2f, %.2f, %.2f]" % (z_dsr[0], z_dsr[1], z_dsr[2]))
                print("pos. error e = [%.2f, %.2f]" % (e[0], e[1]))
                print("u1 = [%.2f, %.2f]" % (u1[0], u1[1]))
                print("u2 = [%.2f, %.2f]" % (u2[0], u2[1]))
                print("[e_proj_u1, e_proj_u2]  = [%.2f, %.2f]" % (e_proj_u1, e_proj_u2))

        self.status = new_status

        if not self._goal_pose_reached_announced:
            rospy.logdebug_throttle(0.5, "[err_dist_norm,  cmd_v] = [%.2f, %.2f]" % (err_dist_norm, cmd_v))
            rospy.logdebug_throttle(0.5, "[err_angle_norm, cmd_w] = [%.2f, %.2f]" % (err_angle_norm, cmd_w))
            rospy.logdebug_throttle(0.5, "after func call status = --------------------------- %s" % self.status)

        self.warning_msg = msg

        return cmd_v, cmd_w


class UnicycleControllerWrapper:
    def __init__(self, config_dict=None):
        """ Init UnicycleControllerWrapper class.

            This controller subscribes:
                odom (from localization)
                desired robot states (from high level controller, i.e., ref_gvn)
            Publish:
                desired velocity / body twist 
        """
        self.preprocessor = None
        self.core = None
        self.postprocessor = None

        # loading external config parameters
        self._config_dict = config_dict
      
        # -------------------- Constants -------------------------
 
        self.ctrl_params = self._config_dict["ctrl_params"]
        self.ctrl_limits = self._config_dict["ctrl_limits"]

        rospy.logwarn("self.ctrl_params %s" % self.ctrl_params)
        rospy.logwarn("self.ctrl_limits %s" % self.ctrl_limits)

        # ------------------- Init Modules  --------------------
        self.init_preprocessor()
        self.init_core()
        self.init_postprocessor()

        # set numpy array console print precision = 2
        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})
        rospy.loginfo("UNICYCLE CONTROL NODE INIT SUCCESSFUL!")


    def get_2d_loc(self, xvec):
        """
        Get 2d location from robot states, or governor states
        """
        return xvec[0:self._nPath]

    def init_preprocessor(self):
        """
        Init preprocessor of node.
        """
        self.preprocessor = UnicycleControlPreprocess()

    def init_core(self):
        """
        Init unicycle controller. For each controller the interface might be different. 
        More details in controller itself.
        """
        self.core = ConeController(ctrl_params=self.ctrl_params)
        
    def init_postprocessor(self):
        """
        Init preprocessor of node.
        """
        self.postprocessor = UnicycleControlPostprocess(ctrl_limits=self.ctrl_limits)
        self.cmd_pub = self.postprocessor.cmd_vel_pub

    def show_debug_info(self, z, z_dsr, v, w):
        """
        Display more debug info. 
        """
        err_dist_norm = np.linalg.norm(z_dsr[0:2] - z[0:2])
        err_angle_norm = np.abs(np.rad2deg(wrap_angle_pmp(z_dsr[2] - z[2])))
        rospy.logdebug_throttle(1, "[unicycle controller update]")
        rospy.logdebug_throttle(1, "z =     [%.2f, %.2f, %.2f]" % (z[0], z[1], z[2]))
        rospy.logdebug_throttle(1, "z_dsr = [%.2f, %.2f, %.2f]" % (z_dsr[0], z_dsr[1], z_dsr[2]))

        if not self.core._goal_pose_reached_announced:
            msg1 = "[llc node] [e_dist, e_angle (deg)] = [%.3f, %.3f]" % (err_dist_norm, err_angle_norm)
            msg1 += "\t[v, w] = [%.2f, %.2f]" % (v, w)
            rospy.loginfo_throttle(1, msg1)


    def update(self):
        """
        Update loop as follows:
            1. collect latest data from preprocessor (callback automatically)
            2. execuate update loop using core
            3. sending command to downstream via post-processor
        """

        z = self.preprocessor._np_z
        z_dsr = self.preprocessor._np_z_dsr
        
        # control gain boost from directional distance metric / euclidean distance metric
        sddm_boost = 1.0
            
        v, w = self.core.generate_control(z=z, z_dsr=z_dsr)

        # show debug info in ros terminal
        # self.show_debug_info(z=z, z_dsr=z_dsr, v=v, w=w)
        # send out velocity command
        self.postprocessor.send_cmd(v_dsr=v, w_dsr=w, clip_ctrl=True)


if __name__ == '__main__':
    try:
        rospy.init_node('unicycle_controller')
        rospy.loginfo("[unicycle_controller] Started!\n")

        # loading parameters
        ctrl_freq = rospy.get_param("~ctrl_freq", 50.0)
        kv = rospy.get_param("~kv", 0.5)
        kw = rospy.get_param("~kw", 1.5)

        # control limit
        v_min = rospy.get_param("~v_min", -0.5)
        v_max = rospy.get_param("~v_max", 2.0)

        w_min = rospy.get_param("~w_min", 1.0)
        w_max = rospy.get_param("~w_max", -1.0)

        config_dict = {
            'ctrl_params': {'kv': kv, 'kw': kw, 'kephi': 0.6, 'kdphi': 0.2},
            'ctrl_limits': {'v_min': v_min, 'v_max': v_max, 'w_min': w_min, 'w_max': w_max}
        }

        unicycle_controller = UnicycleControllerWrapper(config_dict)
        rate = rospy.Rate(ctrl_freq)

        while not rospy.is_shutdown():
            unicycle_controller.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

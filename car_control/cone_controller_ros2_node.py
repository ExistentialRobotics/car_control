#!/usr/bin/env python3
""" Low level velocity controller for unicycle-like robot.

Interfaces:
    Input:
            desired robot states (z*) from ref_gvn node z* = zg
            current robot states (z) from odometry
    Output:
            velocity command for mobile platform or simulated dynamics

"""

from __future__ import print_function

import rclpy
from rclpy.node import Node

import numpy as np
import time
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def clip(x, x_min, x_max):
    """
    clip x in [x_min, x_max]
    """
    if x < x_min:
        x = x_min
    if x > x_max:
        x = x_max
    return x


def wrap_angle_pmp(angle_vec):
    """
    npla.normalize angle in radian to [-pi, pi)
    angle_vec: angle description in radian
    """
    angle_vec = (angle_vec + np.pi) % (2 * np.pi) - np.pi
    return angle_vec


class ConeController:
    """
    Cone controller from Omur's technical report. Using this to compute
    velocity control signal given current and desired robot states.
    """

    # Running status table, higher number better status
    NORMAL = 0
    GOAL_LOC_REACHED = 1
    GOAL_POSE_REACHED = 2

    def __init__(self, ctrl_params, mode="cone"):
        """
        Init cone controller
        Input:
            @ctrl_params: controller design parameters
        """
        if ctrl_params is not None:
            # controller design parameters
            self.kv = ctrl_params["kv"]
            self.kw = ctrl_params["kw"]
            self.dd_ka = ctrl_params["dd_ka"]
            self.dd_kb = ctrl_params["dd_kb"]
        else:
            print("[CarController] use default params")
            self.kv = 0.5
            self.kw = 1.5
            self.dd_ka = 8.0
            self.dd_kb = -1.5

        if mode not in ["cone", "dd"]:
            mode = "cone"
            print("[CarController] incorrect mode, set to cone")

        self.mode = mode
        self.debug_msg = ""
        self.info_msg = "[CarController] started with " + self.mode + " mode"
        self.status = ConeController.NORMAL
        self._goal_pose_reached_announced = False

    def generate_control(self, z, z_dsr,
                         eps_dist=0.1,
                         eps_dist_reset=0.3,
                         eps_angle=0.05,
                         eps_angle_reset=0.2,
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
        # Start with zero controls
        cmd_v = 0.0
        cmd_w = 0.0

        # initial check
        if not self._goal_pose_reached_announced:
            self.info_msg += "\n[car controller] current status = --------------------------- %s \n" % self.status
            if self.status == ConeController.GOAL_POSE_REACHED:
                self.info_msg += "[car controller] GOAL POSE REACHED !! \n"
                self._goal_pose_reached_announced = True

        # error values
        e = z_dsr[0:2] - z[0:2]
        err_dist_norm = np.linalg.norm(e)
        err_angle = wrap_angle_pmp(z_dsr[2] - z[2])
        err_angle_norm = np.abs(err_angle)

        # ----------------------- Finite State Machine -----------------------
        new_status = self.status

        # Hysteresis state jump to combat against noise
        # start with pose_reached status
        if self.status == ConeController.GOAL_POSE_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeController.NORMAL
                self.debug_msg += "\n[car controller] status [down] [pose --> normal] triggered by [dist] err"
                self.debug_msg += ": dist err > eps_dist_reset (%.3f > %.3f) \n" % (err_dist_norm, eps_dist_reset)
                self._goal_pose_reached_announced = False
            # distance error <= eps_dist_reset
            else:
                if err_angle_norm > eps_angle_reset:
                    new_status = ConeController.GOAL_LOC_REACHED
                    self.debug_msg += "\n[car controller] status [down] [pose --> loc] triggered by [angle] err"
                    self.debug_msg += ": angle err > eps_angle_reset (%.3f > %.3f) \n" % (err_angle_norm, eps_angle_reset)
                    self._goal_pose_reached_announced = False
                # distance error <= eps_dist_reset, angle error <= eps_angle_reset
                else:
                    # remain at POSE_REACHED
                    pass

        # start with loc_reached status
        if self.status == ConeController.GOAL_LOC_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeController.NORMAL
                self.debug_msg += "\n[car controller] status [down] [loc --> normal] triggered by [dist] err"
                self.debug_msg += ": dist err > eps_dist_reset (%.3f > %.3f) \n" % (err_dist_norm, eps_dist_reset)
            # distance error <= eps_dist_reset
            else:
                if err_angle_norm <= eps_angle:
                    new_status = ConeController.GOAL_POSE_REACHED
                    self.debug_msg += "\n[car controller] status [ up ] [loc --> pose]"
                    self.debug_msg += ": |angle err| <= eps_angle (%.3f < %.3f) \n" % (err_angle_norm, eps_angle)
                # distance error <= eps_dist_reset, angle error > eps_angle
                else:
                    # remain at LOC_REACHED
                    pass

        # start with normal status
        if self.status == ConeController.NORMAL:
            if err_dist_norm <= eps_dist:
                new_status = ConeController.GOAL_LOC_REACHED
                self.debug_msg += "\n[car controller] status [ up ] [normal --> loc]"
                self.debug_msg += ": dist err > eps_dist (%.3f > %.3f) \n" % (err_dist_norm, eps_dist)
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
            self.info_msg += "[llc = propotional controller] self.kw = %.2f, err_angle = %.2f, cmd_w = %.2f \n" % (self.kw, err_angle, cmd_w)
        # cone controller
        if new_status == ConeController.NORMAL:
            self.info_msg += "[llc  = car controller] active with " + self.mode + " mode \n"
            # ------------------ normal case  ----------------
            if self.mode == "dd":
                theta = z[2]
                u1 = np.array([np.cos(theta), np.sin(theta)])  # heading direction
                u2 = np.array([-np.sin(theta), np.cos(theta)])  # R_ccw(pi/2) * u1

                e_proj_u1 = np.inner(u1, e)
                e_proj_u2 = np.inner(u2, e)

                cmd_v = self.kv * max(0, np.linalg.norm(e))
            else:
                theta = z[2]
                u1 = np.array([np.cos(theta), np.sin(theta)])  # heading direction
                u2 = np.array([-np.sin(theta), np.cos(theta)])  # R_ccw(pi/2) * u1

                e_proj_u1 = np.inner(u1, e)
                e_proj_u2 = np.inner(u2, e)

                cmd_v = self.kv * max(0, e_proj_u1)

            # when close to goal, the angle error is sensitive due to atan2 discontinuity, apply angular velocity scaling
            if np.abs(e_proj_u1) < 2.0 * eps_dist:
                cmd_w = angular_velocity_sf * self.kw * np.arctan2(e_proj_u2, e_proj_u1)
                self.info_msg += "[car controller] close too goal \n"
            else:
                if self.mode == "dd":
                    alpha = wrap_angle_pmp(np.arctan2(e[1], e[0]) - theta)
                    beta = wrap_angle_pmp(-z_dsr[2] - alpha)
                    cmd_w = self.dd_ka * alpha + self.dd_kb * beta
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
            self.info_msg += "[err_dist_norm,  cmd_v] = [%.2f, %.2f] \n" % (err_dist_norm, cmd_v)
            self.info_msg += "[err_angle_norm, cmd_w] = [%.2f, %.2f] \n" % (err_angle_norm, cmd_w)
            self.info_msg += "[car controller] after func call status = --------------------------- %s \n" % self.status
        else:
            self.info_msg += "[car controller] GOAL POSE REACHED !! \n"

        return cmd_v, cmd_w


class ConeControllerPre:
    def __init__(self, node: Node):
        # Ros node from wrapper
        self.node = node

        # Declare parameters
        node.declare_parameter('goal_topic', '/simple_goal')
        node.declare_parameter('goal_type', 'poseS')
        node.declare_parameter('odom_topic', '/odom')
        node.declare_parameter('odom_type', 'poseS')

        # Read parameters
        _goal_topic = node.get_parameter('goal_topic').get_parameter_value().string_value
        _goal_type = node.get_parameter('goal_type').get_parameter_value().string_value
        _odom_topic = node.get_parameter('odom_topic').get_parameter_value().string_value
        _odom_type = node.get_parameter('odom_type').get_parameter_value().string_value

        # Quality of service profile
        qos_profile = QoSProfile(depth=100)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Create Subscribers
        if _odom_type == "poseS":
            self._odom_sub = node.create_subscription(PoseStamped, _odom_topic, self._odom_callback, qos_profile)
            node.get_logger().warn("Odometry message type: [PoseStamped]")
        else:
            self._odom_sub = node.create_subscription(Odometry, _odom_topic, self._odom_callback, qos_profile)
            node.get_logger().warn("Odometry message type: [Odometry]")

        if _goal_type == "poseS":
            self._goal_sub = node.create_subscription(PoseStamped, _goal_topic, self._goal_callback, qos_profile)
            node.get_logger().warn("Goal message type: [PoseStamped]")
        else:
            self._goal_sub = node.create_subscription(Pose2D, _goal_topic, self._goal_callback, qos_profile)
            node.get_logger().warn("Goal message type: [Pose2D]")

        # Upstream status variables
        self.upstream_data_ready = False

        # Upstream data container
        self.np_z = None
        self.np_z_dsr = None

        node.get_logger().info("[Car Controller Preprocessor Created]  \n")

    def check_upstream_data(self):
        """ check whether upstream data container are loaded/initialized correctly"""
        status = True

        # robot state z
        if self.np_z is None:
            status = False
            self.node.get_logger().info("[car controller] waiting zvec init...", throttle_duration_sec=1.0)

        # desired robot state z*, for ref_gvn high level controller z* = zg
        if self.np_z_dsr is None:
            status = False
            self.node.get_logger().info("[car controller] waiting zvec_dsr init...", throttle_duration_sec=1.0)

        if status:
            self.upstream_data_ready = True
            self.node.get_logger().info("\n[car controller] all upstream data initialized !")

    def _odom_callback(self, msg_odom):
        if isinstance(msg_odom, Odometry):
            pose = msg_odom.pose.pose
        elif isinstance(msg_odom, PoseStamped):
            pose = msg_odom.pose
        else:
            self.node.get_logger().warn("Received unknown odometry message type")
            return

        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion_sxyz)
        self.np_z = np.array([pose.position.x, pose.position.y, yaw])

        return

    def _goal_callback(self, msg_pose):
        if isinstance(msg_pose, Pose2D):
            self.np_z_dsr = np.array([msg_pose.x, msg_pose.y, msg_pose.theta])
        elif isinstance(msg_pose, PoseStamped):
            quaternion_sxyz = [msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z,
                               msg_pose.pose.orientation.w]
            (_, _, theta) = euler_from_quaternion(quaternion_sxyz)
            self.np_z_dsr = np.array([msg_pose.pose.position.x, msg_pose.pose.position.y, theta])
        else:
            self.node.get_logger().warn("Received unknown setpoint message type")

        return


class ConeControllerPost:
    """
    Unicycle Controller Post-process Module. Responsible for:
        1) create downstream publisher interface
        2) optional state transformation, e.g., linear->nonlinear, polar->cartesian
        3) clip control with repsect to hardware limits
    """
    def __init__(self, node: Node, ctrl_limits=None):
        self.node = node
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self._body_twist = Twist()
        if ctrl_limits is not None:
            self.ctrl_limits = ctrl_limits
        node.get_logger().info("[Car Controller Post-processor initialized]")

    def send_cmd(self, v_dsr: float, w_dsr: float, clip_ctrl=False, debug=False):
        if debug:
            self.node.get_logger().warn("Input body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr),
                                        throttle_duration_sec=0.5)

        if clip_ctrl and self.ctrl_limits is not None:
            v_dsr = clip(v_dsr, self.ctrl_limits['v_min'], self.ctrl_limits['v_max'])
            w_dsr = clip(w_dsr, self.ctrl_limits['w_min'], self.ctrl_limits['w_max'])

        if debug:
            self.node.get_logger().warn("Output body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr),
                                        throttle_duration_sec=0.5)

        self._body_twist.linear.x = v_dsr
        self._body_twist.angular.z = w_dsr
        self.cmd_vel_pub.publish(self._body_twist)

        return


class ConeControllerNode(Node):
    def __init__(self):
        """ Init UnicycleControllerWrapper class.

            This controller subscribes:
                odom (from localization)
                desired robot states (from high level controller, i.e., ref_gvn)
            Publish:
                desired velocity / body twist
        """
        super().__init__('car_controller')
        self.get_logger().info("[car_controller] Started!")

        # Publish rate parameters
        self.declare_parameter('ctrl_freq', 50.0)
        self.declare_parameter('mode', "cone")
        mode = self.get_parameter('mode').get_parameter_value().string_value
        ctrl_freq = self.get_parameter('ctrl_freq').get_parameter_value().double_value

        # Declare parameters
        self.declare_parameter('kv', 0.5)
        self.declare_parameter('kw', 1.5)
        self.declare_parameter('dd_ka', 8.0)
        self.declare_parameter('dd_kb', -1.5)

        # Control limit
        self.declare_parameter('v_min', -0.5)
        self.declare_parameter('v_max', 2.0)

        self.declare_parameter('w_min', -1.0)
        self.declare_parameter('w_max', 1.0)

        # Retrieve parameter values
        kv = self.get_parameter('kv').get_parameter_value().double_value
        kw = self.get_parameter('kw').get_parameter_value().double_value
        dd_ka = self.get_parameter('dd_ka').get_parameter_value().double_value
        dd_kb = self.get_parameter('dd_kb').get_parameter_value().double_value

        v_min = self.get_parameter('v_min').get_parameter_value().double_value
        v_max = self.get_parameter('v_max').get_parameter_value().double_value
        w_min = self.get_parameter('w_min').get_parameter_value().double_value
        w_max = self.get_parameter('w_max').get_parameter_value().double_value

        # Compose parameter container
        ctrl_params = {'kv': kv, 'kw': kw, 'kephi': 0.6, 'kdphi': 0.2, 'dd_ka': dd_ka, 'dd_kb': dd_kb}
        ctrl_limits = {'v_min': v_min, 'v_max': v_max, 'w_min': w_min, 'w_max': w_max}

        # Display controller settings
        self.get_logger().warn(f"self.ctrl_params {ctrl_params}")
        self.get_logger().warn(f"self.ctrl_limits {ctrl_limits}")

        # Load modules
        self.pre = ConeControllerPre(self)
        self.core = ConeController(ctrl_params, mode)
        self.post = ConeControllerPost(self, ctrl_limits)

        # set numpy array console print precision = 2
        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})
        self.get_logger().info("CAR CONTROL NODE INIT SUCCESSFUL!")

        # setup fix interval update
        timer_period = 1 / ctrl_freq
        self.timer = self.create_timer(timer_period, self.update)

    def update(self):
        """
        Update loop as follows:
            1. collect latest data from preprocessor (callback automatically)
            2. execuate update loop using core
            3. sending command to downstream via post-processor
        """
        if not self.pre.upstream_data_ready:
            self.pre.check_upstream_data()
            time.sleep(1.0)
        else:
            # Get data from subscribers
            z = self.pre.np_z
            z_dsr = self.pre.np_z_dsr

            # Calculate controls
            v, w = self.core.generate_control(z=z, z_dsr=z_dsr)

            # Logging
            self.get_logger().info(self.core.info_msg, throttle_duration_sec=1)
            self.core.info_msg = ""

            if not self.core.debug_msg == "":
                self.get_logger().info(self.core.debug_msg)
                self.core.debug_msg = ""

            # send out velocity command
            self.post.send_cmd(v_dsr=v, w_dsr=w, clip_ctrl=True)


def main(args=None):
    rclpy.init(args=args)
    llc_node = ConeControllerNode()
    try:
        rclpy.spin(llc_node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down")
        pass
    finally:
        llc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

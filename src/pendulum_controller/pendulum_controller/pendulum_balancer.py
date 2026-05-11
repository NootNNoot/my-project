#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


def wrap_to_pi(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


class AdaptiveLQRBalancer(Node):
    def __init__(self):
        super().__init__("adaptive_lqr_balancer")

        self.arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.pend_joint_names = ["pendulum_joint_2"]

        self.positions = {}
        self.velocities = {}

        self.latest_time = 0.0
        self.robot_description = None
        self.kdl_ready = False
        self.gravity_joint_names = []

        self.prev_tau_balance = np.zeros(6)
        self.prev_tau_lqr = np.zeros(6)

        self.last_log_time = 0.0
        self.hold_q_nom = None

        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tool_link", "pendulum_link_2")
        self.declare_parameter("control_rate_hz", 100.0)

        self.declare_parameter("torque_limit", 100.0)
        self.declare_parameter("gravity_scale", 0.75)
        self.declare_parameter("arm_damping_scale", 2.0)

        self.declare_parameter("use_fixed_hold_pose", True)
        self.declare_parameter("hold_joint_positions", [0.0, 0.79, 0.79, 0.0, 0.0, 0.0])
        self.declare_parameter("hold_kp", [2.0, 10.0, 10.0, 0.8, 0.8, 0.0])

        self.declare_parameter("pendulum_upright_angle", 0.0)

        self.declare_parameter("balance_scale", 0.2)

        self.declare_parameter("kp_pend", 8.0)
        self.declare_parameter("kd_pend", 4.0)
        self.declare_parameter("kp_j6", 1.0)
        self.declare_parameter("kd_j6", 3.0)
        self.declare_parameter("balance_alpha", 0.08)

        self.declare_parameter("lqr_gain_path", "")
        self.declare_parameter("lqr_scale", 0.2)
        self.declare_parameter("catch_pend_angle", 0.35)
        self.declare_parameter("catch_pend_vel", 3.0)
        self.declare_parameter("use_lqr", True)

        self.torque_limit = float(self.get_parameter("torque_limit").value)
        self.lqr_K = None

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )

        robot_desc_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.robot_desc_sub = self.create_subscription(
            String,
            "/robot_description",
            self.robot_description_callback,
            robot_desc_qos,
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/arm_effort_controller/commands", 10
        )

        self.load_lqr_gain()

        rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Adaptive LQR balancer started.")

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.velocities[name] = msg.velocity[i]

    def clock_callback(self, msg):
        self.latest_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def robot_description_callback(self, msg):
        if self.robot_description is not None:
            return

        self.robot_description = msg.data
        self.setup_kdl_gravity()

    def setup_kdl_gravity(self):
        try:
            robot = URDF.from_xml_string(self.robot_description)
            ok, tree = treeFromUrdfModel(robot)

            if not ok:
                self.get_logger().error("Failed to build KDL tree.")
                return

            base_link = self.get_parameter("base_link").value
            tool_link = self.get_parameter("tool_link").value

            self.kdl_chain = tree.getChain(base_link, tool_link)

            self.gravity_joint_names = []
            for i in range(self.kdl_chain.getNrOfSegments()):
                seg = self.kdl_chain.getSegment(i)
                joint = seg.getJoint()

                if joint.getType() != PyKDL.Joint.Fixed:
                    self.gravity_joint_names.append(joint.getName())

            self.get_logger().info(
                f"KDL gravity chain: base={base_link}, tool={tool_link}, "
                f"joints={self.gravity_joint_names}"
            )

            self.kdl_dyn = PyKDL.ChainDynParam(
                self.kdl_chain,
                PyKDL.Vector(0.0, 0.0, -9.81),
            )

            self.kdl_ready = True
            self.get_logger().info("KDL gravity compensation ready.")

        except Exception as ex:
            self.kdl_ready = False
            self.get_logger().error(f"KDL setup failed: {ex}")

    def load_lqr_gain(self):
        path = self.get_parameter("lqr_gain_path").value

        if path == "":
            self.get_logger().warn("No lqr_gain_path provided. LQR disabled.")
            return

        try:
            self.lqr_K = np.load(path)
            self.get_logger().info(f"Loaded LQR gain from {path}, shape={self.lqr_K.shape}")
        except Exception as ex:
            self.lqr_K = None
            self.get_logger().error(f"Failed to load LQR gain: {ex}")

    def have_full_state(self):
        names = self.arm_joint_names + self.pend_joint_names
        return all(n in self.positions for n in names) and all(n in self.velocities for n in names)

    def get_ordered_state(self):
        q_arm = np.array([self.positions[n] for n in self.arm_joint_names], dtype=float)
        q_pend = np.array([self.positions[n] for n in self.pend_joint_names], dtype=float)
        dq_arm = np.array([self.velocities[n] for n in self.arm_joint_names], dtype=float)
        dq_pend = np.array([self.velocities[n] for n in self.pend_joint_names], dtype=float)
        return q_arm, q_pend, dq_arm, dq_pend

    def compute_gravity_torque(self, q_arm=None):
        if not self.kdl_ready:
            return np.zeros(6)

        try:
            q_kdl = PyKDL.JntArray(len(self.gravity_joint_names))

            for i, name in enumerate(self.gravity_joint_names):
                if name in self.positions:
                    q_kdl[i] = float(self.positions[name])
                elif q_arm is not None and name in self.arm_joint_names:
                    arm_idx = self.arm_joint_names.index(name)
                    q_kdl[i] = float(q_arm[arm_idx])
                else:
                    self.get_logger().warn(
                        f"Missing joint state for gravity joint '{name}'. Returning zero GC."
                    )
                    return np.zeros(6)

            g_kdl = PyKDL.JntArray(len(self.gravity_joint_names))
            self.kdl_dyn.JntToGravity(q_kdl, g_kdl)

            tau_full = np.array(
                [g_kdl[i] for i in range(len(self.gravity_joint_names))],
                dtype=float,
            )

            tau_arm = np.zeros(6)

            for i, name in enumerate(self.gravity_joint_names):
                if name in self.arm_joint_names:
                    arm_idx = self.arm_joint_names.index(name)
                    tau_arm[arm_idx] = tau_full[i]

            return tau_arm

        except Exception as ex:
            self.get_logger().warn(f"Gravity torque failed: {ex}")
            return np.zeros(6)

    def get_nominal_arm_state(self, current_q_arm):
        use_fixed = bool(self.get_parameter("use_fixed_hold_pose").value)

        if use_fixed:
            q_nom = np.array(
                self.get_parameter("hold_joint_positions").value,
                dtype=float,
            )

            if len(q_nom) != 6:
                self.get_logger().warn("hold_joint_positions must have 6 values.")
                q_nom = current_q_arm.copy()

            return q_nom, np.zeros(6)

        if self.hold_q_nom is None:
            self.hold_q_nom = current_q_arm.copy()
            self.get_logger().info(
                f"Holding current arm pose as nominal: {np.round(self.hold_q_nom, 3)}"
            )

        return self.hold_q_nom.copy(), np.zeros(6)

    def make_error_state(self, q_arm, q_pend, dq_arm, dq_pend, q_nom, dq_nom):
        upright = float(self.get_parameter("pendulum_upright_angle").value)

        return np.array([
            wrap_to_pi(q_arm[0] - q_nom[0]),
            wrap_to_pi(q_arm[1] - q_nom[1]),
            wrap_to_pi(q_arm[2] - q_nom[2]),
            wrap_to_pi(q_arm[3] - q_nom[3]),
            wrap_to_pi(q_arm[4] - q_nom[4]),
            wrap_to_pi(q_arm[5] - q_nom[5]),

            wrap_to_pi(q_pend[0] - upright),

            dq_arm[0] - dq_nom[0],
            dq_arm[1] - dq_nom[1],
            dq_arm[2] - dq_nom[2],
            dq_arm[3] - dq_nom[3],
            dq_arm[4] - dq_nom[4],
            dq_arm[5] - dq_nom[5],

            dq_pend[0],
        ], dtype=float)

    def compute_base_controller(self, q_arm, tau_gc, dq_arm):
        gscale = float(self.get_parameter("gravity_scale").value)
        dscale = float(self.get_parameter("arm_damping_scale").value)

        damping = np.array([1.0, 4.0, 3.0, 0.5, 0.5, 0.3], dtype=float) * dscale

        q_nom = np.array(
            self.get_parameter("hold_joint_positions").value,
            dtype=float,
        )

        kp_hold = np.array(
            self.get_parameter("hold_kp").value,
            dtype=float,
        )

        if len(kp_hold) != 6:
            kp_hold = np.array([2.0, 10.0, 10.0, 0.8, 0.8, 0.0], dtype=float)

        tau_hold = -kp_hold * (q_arm - q_nom)

        return (
            gscale * tau_gc.copy()
            - damping * dq_arm
            + tau_hold
        )

    def compute_pd_balance_torque(self, x_err):
        tau = np.zeros(6)

        joint6_err = x_err[5]
        pend_err = x_err[6]
        joint6_vel = x_err[12]
        pend_vel = x_err[13]

        kp_pend = float(self.get_parameter("kp_pend").value)
        kd_pend = float(self.get_parameter("kd_pend").value)
        kp_j6 = float(self.get_parameter("kp_j6").value)
        kd_j6 = float(self.get_parameter("kd_j6").value)

        u = (
            -kp_pend * pend_err
            -kd_pend * pend_vel
            -kp_j6 * joint6_err
            -kd_j6 * joint6_vel
        )

        tau[0] = 0.10 * u
        tau[1] = 0.30 * u
        tau[2] = 0.30 * u
        tau[5] = 0.05 * u

        alpha = float(self.get_parameter("balance_alpha").value)
        tau = alpha * tau + (1.0 - alpha) * self.prev_tau_balance
        self.prev_tau_balance = tau.copy()

        return tau

    def make_lqr_state(self, q_arm, q_pend, dq_arm, dq_pend, q_nom):
        x = np.zeros(14)

        x[0] = wrap_to_pi(q_arm[0] - q_nom[0])
        x[1] = wrap_to_pi(q_arm[1] - q_nom[1])
        x[2] = wrap_to_pi(q_arm[2] - q_nom[2])
        x[3] = wrap_to_pi(q_arm[3] - q_nom[3])
        x[4] = wrap_to_pi(q_arm[4] - q_nom[4])
        x[5] = wrap_to_pi(q_arm[5] - q_nom[5])

        x[6] = wrap_to_pi(q_pend[0])

        x[7] = dq_arm[0]
        x[8] = dq_arm[1]
        x[9] = dq_arm[2]
        x[10] = dq_arm[3]
        x[11] = dq_arm[4]
        x[12] = dq_arm[5]

        x[13] = dq_pend[0]

        return x

    def is_near_upright(self, q_pend, dq_pend):
        catch_angle = float(self.get_parameter("catch_pend_angle").value)
        catch_vel = float(self.get_parameter("catch_pend_vel").value)

        pend_err = wrap_to_pi(q_pend[0])
        pend_vel = dq_pend[0]

        return abs(pend_err) < catch_angle and abs(pend_vel) < catch_vel

    def compute_lqr_torque(self, q_arm, q_pend, dq_arm, dq_pend, q_nom):
        if self.lqr_K is None:
            return np.zeros(6)

        x = self.make_lqr_state(q_arm, q_pend, dq_arm, dq_pend, q_nom)
        tau = -self.lqr_K @ x

        alpha = float(self.get_parameter("lqr_alpha").value) if self.has_parameter("lqr_alpha") else 0.2
        tau = alpha * tau + (1.0 - alpha) * self.prev_tau_lqr
        self.prev_tau_lqr = tau.copy()

        scale = float(self.get_parameter("lqr_scale").value)
        return scale * tau

    def publish_tau(self, tau_cmd, label=""):
        tau_cmd = np.array(tau_cmd, dtype=float)
        before_clip = tau_cmd.copy()

        tau_cmd = np.clip(tau_cmd, -self.torque_limit, self.torque_limit)

        msg = Float64MultiArray()
        msg.data = [float(x) for x in tau_cmd.tolist()]
        self.cmd_pub.publish(msg)

        if self.latest_time - self.last_log_time > 1.0:
            self.last_log_time = self.latest_time
            self.get_logger().info(
                f"{label} | before_clip={np.round(before_clip, 2)} | "
                f"published={np.round(tau_cmd, 2)} | "
                f"subs={self.cmd_pub.get_subscription_count()}"
            )

    def control_loop(self):
        if not self.have_full_state():
            return

        if not self.kdl_ready:
            return

        q_arm, q_pend, dq_arm, dq_pend = self.get_ordered_state()

        q_nom, dq_nom = self.get_nominal_arm_state(q_arm)
        x_err = self.make_error_state(q_arm, q_pend, dq_arm, dq_pend, q_nom, dq_nom)

        tau_gc = self.compute_gravity_torque(q_arm)
        tau_base = self.compute_base_controller(q_arm, tau_gc, dq_arm)

        use_lqr = bool(self.get_parameter("use_lqr").value)

        if use_lqr and self.lqr_K is not None and self.is_near_upright(q_pend, dq_pend):
            tau_balance = self.compute_lqr_torque(q_arm, q_pend, dq_arm, dq_pend, q_nom)
            label = "base_plus_lqr"
        else:
            tau_balance = self.compute_pd_balance_torque(x_err)
            label = "base_plus_pd"

        balance_scale = float(self.get_parameter("balance_scale").value)
        tau_cmd = tau_base + balance_scale * tau_balance

        self.publish_tau(tau_cmd, label=label)


def main():
    rclpy.init()
    node = AdaptiveLQRBalancer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
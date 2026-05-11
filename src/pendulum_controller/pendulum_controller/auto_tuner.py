#!/usr/bin/env python3

import math
import random
import time
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def wrap_to_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


class FullAutoTuner(Node):
    def __init__(self):
        super().__init__('full_autotuner')

        self.theta1 = None
        self.theta2 = None
        self.theta1_dot = 0.0
        self.theta2_dot = 0.0

        # Upright balancing position
        self.theta1_target = 0.0
        self.theta2_target = 0.0

        self.best_score = float('inf')
        self.best_gains = None

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.client = self.create_client(
            SetParameters,
            '/pendulum_balancer/set_parameters'
        )

    def joint_state_callback(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        if 'pendulum_joint_1' in name_to_idx:
            i = name_to_idx['pendulum_joint_1']
            self.theta1 = msg.position[i]
            self.theta1_dot = msg.velocity[i] if i < len(msg.velocity) else 0.0

        if 'pendulum_joint_2' in name_to_idx:
            i = name_to_idx['pendulum_joint_2']
            self.theta2 = msg.position[i]
            self.theta2_dot = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def clear_state(self):
        self.theta1 = None
        self.theta2 = None
        self.theta1_dot = 0.0
        self.theta2_dot = 0.0

    def wait_for_state(self, timeout_sec=3.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.theta1 is not None and self.theta2 is not None:
                return True
        return False

    def wait_for_near_reset_pose(self, timeout_sec=2.0, tol=0.15):
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.theta1 is None or self.theta2 is None:
                continue

            e1 = wrap_to_pi(self.theta1 - self.theta1_target)
            e2 = wrap_to_pi(self.theta2 - self.theta2_target)

            if abs(e1) < tol and abs(e2) < tol:
                return True

        return False

    def make_param(self, name: str, value: float) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value)
            )
        )

    def set_gains(self, kp1, kd1, kp2, kd2):
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Could not reach /pendulum_balancer/set_parameters')
            return False

        req = SetParameters.Request()
        req.parameters = [
            self.make_param('kp1', kp1),
            self.make_param('kd1', kd1),
            self.make_param('kp2', kp2),
            self.make_param('kd2', kd2),
        ]

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is None:
            self.get_logger().error('SetParameters call returned no result.')
            return False

        return all(result.successful for result in future.result().results)

    def run_cmd(self, cmd, desc: str, timeout_sec: float = 5.0):
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
                check=False
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'{desc} timed out.')
            return False

        if result.returncode != 0:
            self.get_logger().error(
                f'{desc} failed.\n'
                f'CMD: {" ".join(cmd)}\n'
                f'STDOUT:\n{result.stdout}\n'
                f'STDERR:\n{result.stderr}'
            )
            return False

        return True

    def reset_robot_pose(self):
        return self.run_cmd(
            [
                'gz', 'topic',
                '-t', '/double_pendulum/reset_joints',
                '-m', 'gz.msgs.Boolean',
                '-p', 'data: true'
            ],
            'reset_robot_pose',
            timeout_sec=3.0
        )

    def prepare_trial(self, kp1, kd1, kp2, kd2):
        if not self.set_gains(kp1, kd1, kp2, kd2):
            self.get_logger().error('Failed to set gains.')
            return False

        self.clear_state()

        if not self.reset_robot_pose():
            self.get_logger().error('Failed to reset robot pose.')
            return False

        # Give Gazebo a moment to apply the multi-tick reset
        time.sleep(0.15)

        self.clear_state()
        if not self.wait_for_state(timeout_sec=3.0):
            self.get_logger().error('No joint states after reset.')
            return False

        # Optional: verify pendulum actually starts near upright
        if not self.wait_for_near_reset_pose(timeout_sec=1.5, tol=0.2):
            self.get_logger().warn('Pendulum did not appear near reset pose before scoring.')

        return True

    def score_trial(self, duration=5.0, failure_angle=0.7, grace_sec=0.12):
        start = time.time()
        last = start
        score = 0.0

        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.theta1 is None or self.theta2 is None:
                continue

            now = time.time()
            dt = now - last
            last = now

            e1 = wrap_to_pi(self.theta1 - self.theta1_target)
            e2 = wrap_to_pi(self.theta2 - self.theta2_target)

            score += (e1 * e1 + e2 * e2) * dt

            if (now - start) > grace_sec:
                if abs(e1) > failure_angle or abs(e2) > failure_angle:
                    score += 100.0
                    return score, now - start

        return score, duration

    def tune(self, trials=30):
        if not self.wait_for_state(timeout_sec=10.0):
            self.get_logger().error('Never received initial pendulum joint states.')
            return

        for i in range(trials):
            kp1 = random.uniform(0.5, 5.0)
            kd1 = random.uniform(0.05, 1.5)
            kp2 = random.uniform(0.1, 3.0)
            kd2 = random.uniform(0.01, 1.0)

            self.get_logger().info(f'========== Trial {i + 1}/{trials} ==========')

            if not self.prepare_trial(kp1, kd1, kp2, kd2):
                self.get_logger().error('Preparation failed.')
                continue

            score, survive = self.score_trial(duration=5.0)

            self.get_logger().info(
                f'Trial {i + 1}/{trials}: '
                f'kp1={kp1:.3f}, kd1={kd1:.3f}, kp2={kp2:.3f}, kd2={kd2:.3f} | '
                f'score={score:.4f}, survive={survive:.3f}s'
            )

            if score < self.best_score:
                self.best_score = score
                self.best_gains = (kp1, kd1, kp2, kd2)
                self.get_logger().info(
                    f'NEW BEST -> '
                    f'kp1={kp1:.6f}, kd1={kd1:.6f}, '
                    f'kp2={kp2:.6f}, kd2={kd2:.6f}, '
                    f'best_score={score:.6f}'
                )

        self.get_logger().info('==== FINAL BEST ====')
        if self.best_gains is None:
            self.get_logger().info('No valid trials completed.')
        else:
            kp1, kd1, kp2, kd2 = self.best_gains
            self.get_logger().info(
                f'kp1={kp1:.6f}, kd1={kd1:.6f}, '
                f'kp2={kp2:.6f}, kd2={kd2:.6f}, '
                f'best_score={self.best_score:.6f}'
            )


def main():
    rclpy.init()
    node = FullAutoTuner()
    node.tune(trials=30)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
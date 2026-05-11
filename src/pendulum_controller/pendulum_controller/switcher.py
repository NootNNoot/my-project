import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool


class ControllerSwitcher(Node):

    def __init__(self):
        super().__init__('controller_switcher')

        self.sub = self.create_subscription(
            Bool,
            '/controller_switcher',
            self.switcher_callback,
            10
        )

        self.cli = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for controller manager...")

    def switcher_callback(self, msg: Bool):

        if msg.data:
            self.switch(
                activate=['pendulum_position_controller']
            )
        else:
            self.switch(
                deactivate=['pendulum_position_controller']
            )

    def switch(self, activate=None, deactivate=None):

        req = SwitchController.Request()

        req.activate_controllers = activate or []
        req.deactivate_controllers = deactivate or []

        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True

        future = self.cli.call_async(req)

        future.add_done_callback(self.switch_done_callback)


    def switch_done_callback(self, future):

        result = future.result()

        if result is None:
            self.get_logger().error("Switch failed")
            return

        if result.ok:
            self.get_logger().info("Switch successful")
        else:
            self.get_logger().error("Switch unsuccessful")


def main():
    rclpy.init()

    node = ControllerSwitcher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
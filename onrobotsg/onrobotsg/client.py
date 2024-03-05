import sys
import rclpy
from rclpy.node import Node
from onrobotsg_interfaces.srv import OnrobotSG


class OnrobotSGClient(Node):

    def __init__(self):
        super().__init__("onrobotsg_gripper_client")
        self.cli = self.create_client(OnrobotSG, "gripper_command")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_request(self, width):
        reqMsg = OnrobotSG.Request()
        reqMsg.desiredwidth = width
        self.future = self.cli.call_async(reqMsg)
        return self.future


def main(args=None):
    rclpy.init(args=args)
    clientNode = OnrobotSGClient()
    widthInput = float(sys.argv[1])
    future = clientNode.send_request(widthInput)

    while rclpy.ok():
        rclpy.spin_once(clientNode)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                clientNode.get_logger().info(f"Service call failed {(e,)}")
            else:
                clientNode.get_logger().info(f"Status is : {response.status}")
            break

    clientNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

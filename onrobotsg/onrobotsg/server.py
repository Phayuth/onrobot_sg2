import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from onrobotsg_interfaces.srv import OnrobotSG
from .driver import OnrobotSGDriver


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class OnrobotSGServer(Node):

    def __init__(self):
        super().__init__('onrobotsg_gripper_server')
        self.declare_parameter('ip', '192.168.0.100', ParameterDescriptor(description='IP Address to Gripper Compute Box'))
        self.declare_parameter('port', 502, ParameterDescriptor(description='Port ID'))
        self.declare_parameter('model_id', 4, ParameterDescriptor(description='Model ID: 1:None, 2:SG-a-H (Flower-Like w/o White Tip), 3:SG-a-S (Flower-Like w/ White Tip), 4:SG-b-H (Claw-Like)'))
        self.declare_parameter('gentle', False, ParameterDescriptor(description='Gripping speed is reduced at 12.5mm before the specified target width'))

        self.ip = self.get_parameter('ip').value
        self.port = self.get_parameter('port').value
        self.modelID = self.get_parameter('model_id').value
        self.gentle = self.get_parameter('gentle').value
        self.add_on_set_parameters_callback(self.set_param_cb)

        self.get_logger().info(bcolors.OKGREEN + "Setting Up Connection" + bcolors.ENDC)
        self.sg = OnrobotSGDriver(self.ip, self.port, self.modelID)
        self.sg.set_gentle(self.gentle)
        self.get_logger().info(f"Gripper Width Range : {[self.sg.minWidth, self.sg.maxWidth]} in (mm)")
        self.srv = self.create_service(OnrobotSG, 'gripper_command', self.execute_callback)

    def set_param_cb(self, params):
        for p in params:
            if p.name == 'gentle':
                if p.type_ == p.Type.BOOL:
                    self.get_logger().info(f"Gentle is Setted to {p.value}")
                    self.gentle = p.value
                    self.sg.set_gentle(self.gentle)
                    result = SetParametersResult(successful=True)
                else:
                    result = SetParametersResult(successful=False)
                    result.reason = 'Variable must be BOOL Type'
        return result

    def execute_callback(self, request, response):
        self.get_logger().info(f'Incoming request {[request.desiredwidth]} mm')
        self.sg.set_target(request.desiredwidth)
        self.sg.set_move()
        response.status = "SUCCESS"
        return response

    def close_connection(self):
        self.sg.close_connection()
        self.get_logger().info(bcolors.OKGREEN + "Gripper connection is disconnected" + bcolors.ENDC)


def main(args=None):
    rclpy.init(args=args)
    try:
        serviceNode = OnrobotSGServer()
        rclpy.spin(serviceNode)
    finally:
        serviceNode.close_connection()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from deliverybots_interfaces.srv import SetIntegers

class SetIntegersServer(Node):

    def __init__(self):
        super().__init__('set_integers_server')
        self.srv = self.create_service(SetIntegers, 'set_integers', self.set_integers_callback)

    def set_integers_callback(self, request, response):
        if 0 <= request.input_integer <= 7:
            # Set the integer number within the valid range
            self.input_integer = request.input_integer
            self.get_logger().info('Integer number set to: %d' % self.input_integer)
            response.success = True
            response.output_integer = self.input_integer * 1
            response.message = 'Integer number set successfully'
        else:
            response.output_integer = 0 
            response.success = False
            response.message = 'Integer must be within the range of 0 to 7'
        return response

def main(args=None):
    rclpy.init(args=args)
    set_integers_server = SetIntegersServer()
    rclpy.spin(set_integers_server)
    set_integers_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
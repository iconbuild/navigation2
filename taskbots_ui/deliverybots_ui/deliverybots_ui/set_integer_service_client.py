import rclpy
from rclpy.node import Node
from deliverybots_interfaces.srv import SetIntegers

class SetIntegersClient(Node):

    def __init__(self):
        super().__init__('set_integers_client')
        self.client = self.create_client(SetIntegers, 'set_integers')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = SetIntegers.Request()
        self.request.input_integer = 0

    # def set_integers(self, input_integer):
    #     self.request.input_integer = input_integer
    #     future = self.client.call_async(self.request)
    #     future.add_done_callback(self.set_integers_callback)

    # def set_integers_callback(self, future):
    #     try:
    #         response = future.result()
    #         if response.success:
    #             self.get_logger().info(response.message)
    #         else:
    #             self.get_logger().error(response.message)
    #     except Exception as e:
    #         self.get_logger().error(f'Failed to call service: {e}')


    def set_input_integer(self, input_integer):
        self.request.input_integer = input_integer

    def get_output_integer(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None:
            output_integer = response.output_integer
            self.get_logger().info('Output integer: %d' % output_integer)
            return output_integer
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    set_integers_client = SetIntegersClient()
    
    # Set integer value
    integer_to_set = 0
    #set_integers_client.set_integers(integer_to_set)
     # Example: Set input integer to 5
    set_integers_client.set_input_integer(integer_to_set)
    
    # Call the service and record the output
    output = set_integers_client.get_output_integer()
    print(output)

    rclpy.spin_once(set_integers_client)
    set_integers_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
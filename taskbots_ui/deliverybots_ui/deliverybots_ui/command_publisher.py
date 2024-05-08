import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from deliverybots_interfaces.msg import Commands

class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Commands, 'delivery_command', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Commands()
        msg.command = "load"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.command)



def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    rclpy.spin(command_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

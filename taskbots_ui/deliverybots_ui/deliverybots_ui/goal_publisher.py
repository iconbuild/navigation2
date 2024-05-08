import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int16
from deliverybots_interfaces.msg import Goals


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        # Declare parameter
        self.declare_parameter("pub_goal", 0)

        # get parameter value
        self.pub_goal = self.get_parameter("pub_goal").value

        self.publisher_ = self.create_publisher(Goals, 'delivery_goal', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Goals()
        msg.goal = self.pub_goal
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.goal)



def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    rclpy.spin(goal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

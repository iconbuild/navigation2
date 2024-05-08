#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time


class DockPublisher(Node):
    def __init__(self):
        super().__init__('docking')
        self.speed = 0.1
        self.docked = False
        self.duration = 10 # x seconds

        # Declare parameter
        self.declare_parameter("publish_frequency", 1.0) # Default value is provided
    
        # get parameter value
        self.publish_frequency = self.get_parameter("publish_frequency").value


        # Publish to a topic
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.publish_frequency, self.docking_callback)


    # Callback o publish docking state based limit switch reading from modbus
    def docking_callback(self):
        start_time = time.time()

        while time.time() - start_time <= self.duration and not self.docked: 
            #self.get_logger().info(f'Docking = {True}, start_time ={start_time}, current_time={time.time()}, delta_time = {time.time() - start_time}')
            d_twist = Twist()
            d_twist.linear.x = self.speed
            #time.sleep(1)
            self.publisher.publish(d_twist)

            if time.time() - start_time > self.duration:
                self.docked = True

   
def main(args=None):
    rclpy.init(args=args)
    node = DockPublisher()
    try:
        rclpy.spin(node)
    finally:
        #node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

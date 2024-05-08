#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PointStamped

x = 0.0
y = 0.0
z = 0.0

def callback(msg):
    global x, y, z
    point = PointStamped()
    point.header.stamp = msg.header.stamp
    point.header.frame_id = "/map"
    point.point.x = msg.point.x
    point.point.y = msg.point.y
    point.point.z = msg.point.z
    x = point.point.x
    y = point.point.y
    z = point.point.z
    print("coordinates: x=%f y=%f z=%f" % (x, y, z))

def listener():
    rclpy.init()
    node = rclpy.create_node('waiterbot_listener')
    subscriber = node.create_subscription(
        PointStamped,
        '/clicked_point',
        callback,
        10)
    rclpy.spin(node)


if __name__ == '__main__':
    listener()
#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def stop_callback(msg):
    if msg.data:
        rospy.signal_shutdown("Done...")
    else:
        pass

def dock():
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rospy.init_node('docking')
    rate = rospy.Rate(10)
    rospy.Subscriber('docking_state',Bool,stop_callback)
    # duration = 16.0
    # rospy.Timer(rospy.Duration(duration),stop_callback)
    speed = 0.1
    while not rospy.is_shutdown():
        rospy.loginfo('Docking')
        d_twist = Twist()
        d_twist.linear.x = speed
        pub.publish(d_twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        dock()
    except rospy.ROSInterruptException:
        pass

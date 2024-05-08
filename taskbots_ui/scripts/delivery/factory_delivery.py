# (c) robotics.snowcron.com
# Use: MIT license
 
import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
#from  factory_navigator import FactoryNavigator 
# from  factory_navigator_waypoint import FactoryNavigator   # using Waypoints
from factory_navigator_v2 import FactoryNavigator, HornTopic # This is Experiment code
from tf_test_script import FrameListener

from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

def main(args=None):

    try:  
        rclpy.init(args=args)
        
        executor = MultiThreadedExecutor()
        frame_listener = FrameListener()
        navigator = FactoryNavigator(frame_listener)
        executor.add_node(navigator)
        executor.add_node(frame_listener)
    
        horn_topic = HornTopic()
        executor.add_node(horn_topic)

        try:
            executor.spin()
            # rclpy.spin(navigator)
            # while(True):
            #     bResult = navigator.navigate_to_waypoints_or_charger()
            #     if(not bResult):
            #         break
            #     time.sleep(0.1)
        finally:
            # executor.shutdown()
            navigator.destroy_node()
            frame_listener.destroy_node()
            horn_topic.destroy_node()
 
    finally:
        # Shutdown
        rclpy.shutdown()    
 
 
if __name__ == '__main__':
  main()





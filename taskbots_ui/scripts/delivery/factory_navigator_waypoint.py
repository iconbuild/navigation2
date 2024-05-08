import time  # Time library
 
from rclpy.duration import Duration
import rclpy                                # Python client library for ROS 2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped   # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist         # Velocity command
from sensor_msgs.msg import BatteryState    # Battery status
import time
from datetime import datetime
import random
from nav2_msgs.action import NavigateToPose

# To use the built-in BasicNavigator
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# To use teh modified BasicNavigator
# from basic_navigator import NavigationResult

class FactoryNavigator(Node):
    def __init__(self):
        super().__init__('FactoryNavigator')
        self.nBatteryLevel = 100.0
        timer_period = 0.1
        self.navStatus = "not_started"
        self.navActive = False #  To track if navigation is active at the moment

        self.navigator = BasicNavigator()

        # Set the robot's goal poses: 
        self.goal_poses = []

        # Goal 1 : Load / Unload position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 5.31
        goal_pose.pose.position.y = -26.45
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.67
        goal_pose.pose.orientation.w = -0.73
        self.goal_poses.append(goal_pose)

        # Goal 2 : Load / Unload position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 4.10
        goal_pose.pose.position.y = -21.55
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.99
        goal_pose.pose.orientation.w = -0.04
        self.goal_poses.append(goal_pose)

        # Goal 3 : Load / Unload position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 10.35
        goal_pose.pose.position.y = -14.97
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.15
        goal_pose.pose.orientation.w = -0.98
        self.goal_poses.append(goal_pose)

        # To be able to pass behavior trees 
        # goal_with_bt = NavigateToPose()
        # goal_with_bt.pose.header.frame_id = 'map'
        # goal_with_bt.pose.header.frame_id = 'map'
        # goal_with_bt.pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_with_bt.pose.pose.position.x = 4.10
        # goal_with_bt.pose.pose.position.y = -21.55
        # goal_with_bt.pose.pose.position.z = 0.0
        # goal_with_bt.pose.pose.orientation.x = 0.0
        # goal_with_bt.pose.pose.orientation.y = 0.0
        # goal_with_bt.pose.pose.orientation.z = 0.99
        # goal_with_bt.pose.pose.orientation.w = -0.04
        # goal_with_bt.behavior_tree = '/path/to/bt'

        # Goal 4: Charging Station
        self.goal_charger = PoseStamped()
        self.goal_charger.header.frame_id = 'map'
        self.goal_charger.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_charger.pose.position.x = 17.04
        self.goal_charger.pose.position.y = -14.52
        self.goal_charger.pose.position.z = 0.0
        self.goal_charger.pose.orientation.x = 0.0
        self.goal_charger.pose.orientation.y = 0.0
        self.goal_charger.pose.orientation.z = 0.93
        self.goal_charger.pose.orientation.w = 0.35

        # Batery Full and low thresholds
        self.nBatteryFullThreshold = 100.0
        self.nBatteryLowTreshold = 45


        # After we done recharging, continue from this waypoint
        self.nNextWaypointIdx = 0 #random.randint(0, 2) 

        # Used to publish status
        self.prev_time = datetime.now()

        self.timer = self.create_timer(timer_period, self.navigate_to_waypoints_or_charger)
        self.subscription_battery_status = self.create_subscription(
            BatteryState,
            '/battery',
            self.get_battery_status,
            10)
        # self.publisherBatteryLevel = self.create_publisher(BatteryState, '/battery_status', 10)

    # This is to overload the main function inside BasicNavigator and call this function
    def waitUntilNav2Active(self):
        print('>>> amcl...', end=' ')
        self.navigator._waitForNodeToActivate('amcl')
        print('done.')


        print(">>>_waitForNodeToActivate...", end=" ")
        self.navigator._waitForNodeToActivate('bt_navigator')
        print('done.')

        self.navigator.info('Nav2 is ready for use!')
        return

    def get_battery_status(self, msg):
        self.nBatteryLevel = msg.percentage
        # self.get_logger().info( ">>> Battery Level at %.0d" % (self.nBatteryLevel))

    # def publishBatteryStatus(self):
    #     # self.nBatteryFullThreshold -= 0.1
    #     self.nBatteryFullThreshold += 0.1
    #     msg = BatteryState()
    #     msg.percentage = self.nBatteryFullThreshold
    #     self.publisherBatteryLevel.publish(msg)

       
    def navigate_to_waypoints_or_charger(self):
        # self.publishBatteryStatus()
        
        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        if delta_ms >= 1000:
            print(">>> Battery at %.0d" % (self.nBatteryLevel))
            self.prev_time = now
        

        if(self.navStatus == "not_started"):

            # Set the robot's initial pose if necessary
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 13.0
            initial_pose.pose.position.y = -23.2
            initial_pose.pose.position.z = 0.0
            initial_pose.pose.orientation.x = 0.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = -1.0
            # self.navigator.setInitialPose(initial_pose)
        
            # Activate navigation, if not autostarted. This should be called after setInitialPose()
            # or this will initialize at the origin of the map and update the costmap with bogus readings.
            # If autostart, you should `waitUntilNav2Active()` instead.
            # self.navigator.lifecycleStartup()
            
            # Wait for navigation to fully activate. Use this line if autostart is set to true.
            self.waitUntilNav2Active()
            # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            # self.navigator.setInitialPose(initial_pose)
            self.navStatus = "ready"
      
            # If desired, you can change or load the map as well
            # self.navigator.changeMap('/path/to/map.yaml')
            
            # You may use the self.navigator to clear or obtain costmaps
            # self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
            # global_costmap = self.navigator.getGlobalCostmap()
            # local_costmap = self.navigator.getLocalCostmap()Complete

        elif((self.navStatus == "ready" or (self.navStatus == "navigating" and not self.navActive) ) 
            and self.nBatteryLevel <= self.nBatteryLowTreshold):
            
            print("Battery low")
            self.navStatus = "preparing_to_go_to_charger"
            self.navigator.cancelTask()
            self.navActive = False

        elif(self.navStatus == "ready"):
            # sanity check a valid path exists
            # path = self.navigator.getPathThroughPoses(initial_pose, self.goal_poses)
            self.navigator.followWaypoints(self.goal_poses[self.nNextWaypointIdx:])
            self.navStatus = "navigating"
            self.navActive = False            

        elif(self.navStatus == "navigating"):
            if(not self.navigator.isTaskComplete()):
                feedback = self.navigator.getFeedback()
                self.nNextWaypointIdx = feedback.current_waypoint
                self.navActive = True
                
                if feedback and delta_ms >= 1000:
                    print('Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses)))
            else:
                self.navStatus = "completed"
                self.navActive = False

        elif(self.navStatus == "preparing_to_go_to_charger"):
            print('Cancelling nav. before going to charger')
            if(self.navigator.isTaskComplete()):
                self.navStatus = "going_to_charger"
                self.navigator.goToPose(self.goal_charger)   
            self.navActive = False

        elif(self.navStatus == "going_to_charger"):
            if(self.navigator.isTaskComplete()):
                self.navStatus = "charging"
            self.navActive = False

        elif(self.navStatus == "docking"): # TODO - docking not implementd
            print('DOCKING ...')
            self.navStatus = "docking"
            self.navActive = False  
            if (self.navigator.isTaskComplete()) and self.nBatteryLevel > self.nBatteryLowTreshold:
                self.navStatus = "charging"

        elif(self.navStatus == "charging"):
            print('CHARGING ...')
            self.navStatus = "charging"
            self.navActive = False  
            if (self.navigator.isTaskComplete()) and self.nBatteryLevel > self.nBatteryLowTreshold:
                self.navStatus = "ready"

        elif(self.navStatus == "completed"):
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                self.navStatus = "ready"
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
                self.timer.cancel()
                return False
            elif result == TaskResult.FAILED:
                print('Goal failed!')
                self.timer.cancel()
                return False
            else:
                print('Goal has an invalid return status!')
                print(result)
                self.timer.cancel()
                return False
            self.navActive = False

        return True
         
    # def connect_to_dock(self):  
       
    #     # While the battery is not charging
    #     while this_battery_state.power_supply_status != 1:
      
    #       # Publish the current battery state
    #       self.get_logger().info('NOT CHARGING...')
        
    #       # Send the velocity command to the robot by publishing to the topic
    #       cmd_vel_msg = Twist()
    #       cmd_vel_msg.linear.x = self.linear_velocity
    #       cmd_vel_msg.angular.z = self.angular_velocity
    #       self.publisher_cmd_vel.publish(cmd_vel_msg)      
    #       time.sleep(0.1)
      
    #     # Stop the robot
    #     cmd_vel_msg = Twist()
    #     cmd_vel_msg.linear.x = 0.0
    #     cmd_vel_msg.angular.z = 0.0
    #     self.publisher_cmd_vel.publish(cmd_vel_msg)
      
    #     self.get_logger().info('CHARGING...')
    #     self.get_logger().info('Successfully connected to the charging dock!')
 

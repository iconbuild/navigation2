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
from deliverybots_interfaces.msg  import Commands, Goals
from deliverybots_interfaces.srv import SetIntegers
from std_msgs.msg import String, Int16

# To use the built-in BasicNavigator
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# To use teh modified BasicNavigator
# from basic_navigator import NavigationResult

# Set Integers Client 
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

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        # Declare parameter
        self.declare_parameter("pub_goal", 0)
        # get parameter value
        self.pub_goal = self.get_parameter("pub_goal").value

        self.publisher_ = self.create_publisher(Int16, 'delivery_goal', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int16()
        msg.data = self.pub_goal
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


class FactoryNavigator(Node):
    def __init__(self):
        super().__init__('FactoryNavigator')
        self.nBatteryLevel = 100.0
        timer_period = 0.1
        self.navStatus = "not_started"
        self.navActive = False #  To track if navigation is active at the moment
        self.requested_command = " "
        self.current_goal = 0
        self.set_goal = 0

        self.navigator = BasicNavigator()

        #self.set_integer_client = SetIntegersClient()
        self.delivery_goal_publiser = GoalPublisher()

        # Set the robot's goal poses: 
        self.goal_poses = []

        # Goal 0 : A place holder for doing nothing
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0
        self.goal_poses.append(goal_pose)

        ## ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ## ~~~~~~~~~~~~~Delivery stations ~~~~~~~~~~~~~
        ## ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Goal 1 : Unloading station 1
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

        # Goal 2 : Unloading station 2
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

        # Goal 3 : Unloading station 3
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

        
        # TODO: EVERYTHING below this is the same location
        # Goal 4 : Unloading station 4
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


        # Goal 5 : Unloading station 5
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



        # Goal 6 : Unloading station 6
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

        # Goal 7 : Unloading station 7
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

        ## ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ## ~~~~~Loading + Charging stations ~~~~~~~~~~~
        ## ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Home: Charging Station + Loading Station
        self.goal_load = PoseStamped()
        self.goal_load.header.frame_id = 'map'
        self.goal_load.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_load.pose.position.x = 17.04
        self.goal_load.pose.position.y = -14.52
        self.goal_load.pose.position.z = 0.0
        self.goal_load.pose.orientation.x = 0.0
        self.goal_load.pose.orientation.y = 0.0
        self.goal_load.pose.orientation.z = 0.93
        self.goal_load.pose.orientation.w = 0.35

        # Batery Full and low thresholds
        self.nBatteryFullThreshold = 100.0
        self.nBatteryLowTreshold = 45

        # Used to publish status
        self.prev_time = datetime.now()

        self.timer = self.create_timer(timer_period, self.navigate_to_waypoints_or_charger)
        self.subscription_battery_status = self.create_subscription(
            BatteryState,
            '/battery',
            self.get_battery_status,
            10)
        
        self.subscription_delivery_status = self.create_subscription(
            Commands,
            'delivery_command',
            self.get_delivery_status,
            10)
        
        self.subscription_current_goal = self.create_subscription(
            Int16,
            'delivery_goal',
            self.get_delivery_goal,
            10)

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

    def get_delivery_status(self, msg):
        self.requested_command = msg.command
        #self.get_logger().info(self.requested_command) 

    def get_delivery_goal(self, msg):
        self.current_goal = msg.data
        #self.get_logger().info(">>> Current Goal %.0d" % (self.current_goal)) 
      
    def navigate_to_waypoints_or_charger(self):
        
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
    
      
            # If desired, you can change or load the map as well
            # self.navigator.changeMap('/path/to/map.yaml')
            
            # You may use the self.navigator to clear or obtain costmaps
            # self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
            # global_costmap = self.navigator.getGlobalCostmap()
            # local_costmap = self.navigator.getLocalCostmap()Complete


            self.navStatus = "ready"
      
        elif((self.navStatus == "ready" or (self.navStatus == "navigating" and not self.navActive) ) 
            and self.nBatteryLevel <= self.nBatteryLowTreshold):
            
            # print("Battery low")
            # self.navStatus = "preparing_to_go_to_charger"
            # self.navigator.cancelTask()
            # self.navActive = False
            self.navStatus == "cancel"

        elif(self.navStatus == "cancel"): 

            self.navStatus = "canceling_navigation"
            self.navigator.cancelTask()
            self.navActive = False

        elif(self.navStatus == "canceling_navigation"):
            print('Cancelling nav. before going to charger')
            if(self.navigator.isTaskComplete()):
                self.navStatus == "ready"  
            self.navActive = False

        elif(self.navStatus == "ready"): 

            #integer_to_set = 0
            #self.set_integer_client.set_input_integer(integer_to_set)
            
            # Call the service and record the output
            # self.set_goal = self.set_integer_client.get_output_integer()
            # print(self.set_goal)

            # Wait here for comamnds from user
            print('Waiting for Command: Select goal location')
            if self.current_goal > 0:
                self.navigate_to_goal =  self.goal_poses[self.current_goal]
                self.navStatus = "Reset_goal"
                self.navActive = False  

                
        elif(self.navStatus == "Reset_goal"):   # Reset the goal back to zer for next round
            goal_msg = Int16()
            goal_msg.data = 0
            self.delivery_goal_publiser.publisher_.publish(goal_msg)
            if self.current_goal == 0:  # Goal set back to zer
                self.navigator.goToPose(self.navigate_to_goal)  
                self.navStatus = "navigating"
                self.navActive = False          

        elif(self.navStatus == "navigating"):
            print('Navigating to the goal')
            if(not self.navigator.isTaskComplete()):
                self.navActive = True
            else:
                self.navStatus = "unload"
                self.navActive = False

        elif(self.navStatus == "unload"):

            print('Waiting to complete Unloading ...')
            self.navActive = False
            if True: #TODO : Add condition based on button press
                self.navStatus = "preparing_to_go_to_loading_station"

        elif(self.navStatus == "preparing_to_go_to_loading_station"):
            if(self.navigator.isTaskComplete()):
                self.navStatus = "going_to_loading_station"
                self.navigator.goToPose(self.goal_load)   
            self.navActive = False

        elif(self.navStatus == "going_to_loading_station"):
            print('Navigating to loading station ...')
            if(self.navigator.isTaskComplete()):
                self.navStatus = "docking"
            self.navActive = False

        elif(self.navStatus == "docking"): # TODO - docking not implementd
            print('DOCKING ...')
            self.navStatus = "docking"
            self.navActive = False  
            if (self.navigator.isTaskComplete()) and self.nBatteryLevel > self.nBatteryLowTreshold:
                self.navStatus = "completed"

        elif(self.navStatus == "completed"):
            goal_msg = Int16()
            goal_msg.data = 0
            self.delivery_goal_publiser.publisher_.publish(goal_msg)

            result = self.navigator.getResult()
            if self.current_goal ==0: # Goal reset to 0
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
         
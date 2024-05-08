import time  # Time library
 
from rclpy.duration import Duration
from typing import Tuple
import rclpy                                # Python client library for ROS 2
import rclpy.duration
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped   # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist         # Velocity command
from sensor_msgs.msg import BatteryState    # Battery status
import time
from datetime import datetime
import random
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from deliverybots_interfaces.srv import SetIntegers
from std_msgs.msg import String, Int16, Bool

# To use the built-in BasicNavigator
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import factory_utils as f_util
import tf2_ros
import math
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from nav2_msgs.action import DriveOnHeading
from geometry_msgs.msg import Point


Pose2D = Tuple[float, float, float]

class FrameListener:

    def __init__(self, node):
        self.node = node
        # super().__init__('tf2_frame_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.timer = self.node.create_timer(0.05, self.update_tfs)
        self.left_tf = self.on_timer('charge_left')
        self.right_tf = self.on_timer('charge_right')
        self.charge_tf = self.on_timer('charge_center')
        

    def update_tfs(self):
        tf_temp = self.on_timer('charge_left')
        if tf_temp is not None:
            self.left_tf = tf_temp 
        tf_temp = self.on_timer('charge_right')
        if tf_temp is not None:
            self.right_tf = tf_temp
        tf_temp =  self.on_timer('charge_center')
        if tf_temp is not None:
            self.charge_tf = tf_temp

    def on_timer(self, target_charger: str):
        target_charger_tf = self.get_tf(target_charger)
        return target_charger_tf


    def get_tf(self,target_link: str,
                ref_link=None,
                target_time=None) -> np.ndarray:
            """
            This will provide the transformation of the marker,
            if ref_link is not provided, we will use robot's base_link as ref
            :param now :
            :return : 4x4 homogenous matrix, None if not avail
            """
            if ref_link is None:
                ref_link = 'base_link'
            if target_time is None:
                target_time = rclpy.time.Time()
            try: 
                return f_util.get_mat_from_transfrom_msg( self.tf_buffer.lookup_transform(
                            ref_link, target_link, target_time,
                            rclpy.duration.Duration(seconds=1.0)
                            ) )
            except:
                return None
            
            
# Set Integer server
class SetGoalServer:

    def __init__(self,node):
        self.node = node
        self.srv = self.node.create_service(SetIntegers, 'set_integers', self.set_integers_callback)
        self.input_integer = 0

    def set_integers_callback(self, request, response):
        if 0 <= request.input_integer <= 7: # Set the goal within the valid range
            
            self.input_integer = request.input_integer
            self.node.get_logger().info('Goal set to: %d' % self.input_integer)
            response.success = True
            response.output_integer = self.input_integer * 1
            response.message = 'Goal set successfully'
        else:
            response.output_integer = 0 
            response.success = False
            response.message = 'Goal must be within 0 to 7'
        return response


# Set Integers Client 
class SetGoalClient(Node):

    def __init__(self,executor=None):
        super().__init__('integers_sub_node')
        self.client = self.create_client(SetIntegers, 'set_integers')
         
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = SetIntegers.Request()
        self.executor = executor

    def set_integers(self, input_integer):
        self.request.input_integer = input_integer
        future = self.client.call_async(self.request)
        future.add_done_callback(self.set_integers_callback)

    def set_integers_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')

# Light ON Client 
class LightOnClient(Node):

    def __init__(self,executor=None):
        super().__init__('light_on')
        self.light_on_client = self.create_client(Trigger, '/u_light_on')
        self.executor = executor

        while not self.light_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Light ON service not available, waiting again...')

    def send_request(self):
        request = Trigger.Request()
        future = self.light_on_client.call_async(request)
        rclpy.spin_until_future_complete(self, future,executor=self.executor)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Light OFF Service call failed %r' % (future.exception(),))

# Light OFF Client 
class LightOffClient(Node):

    def __init__(self,executor=None):
        super().__init__('light_off')
        self.light_off_client = self.create_client(Trigger, '/u_light_off')
        self.executor = executor

        while not self.light_off_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Light OFF service not available, waiting again...')

    def send_request(self):
        request = Trigger.Request()
        future = self.light_off_client.call_async(request)
        rclpy.spin_until_future_complete(self, future,executor=self.executor)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Light OFF Service call failed %r' % (future.exception(),))


# Honk Horn Client 
class HornClient(Node):

    def __init__(self,executor=None):
        super().__init__('horn')
        self.horn_client = self.create_client(Trigger, '/honk_horn')
        self.executor = executor
    
        while not self.horn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Horn service not available, waiting again...')
           

    def send_request(self,timeout=None):
        request = Trigger.Request()
        future = self.horn_client.call_async(request)
        if timeout:
            rclpy.spin_until_future_complete(self, future,executor=self.executor,timeout_sec=timeout)
        else:
            rclpy.spin_until_future_complete(self, future,executor=self.executor)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Horn Service call failed %r' % (future.exception(),))

class HornTopic(Node):
    def __init__(self):
        super().__init__('horn_topic_node')
        self.horn_client = self.create_client(Trigger, '/honk_horn')
        qos_horn = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
        self.horn_cb_group = ReentrantCallbackGroup()
        self.horn_listener = self.create_subscription(Bool,'/horn_on',callback=self.cb_horn, qos_profile=qos_horn, callback_group= self.horn_cb_group)

    
    def cb_horn(self,message):
        # self.get_logger().info(f'Toot toot...{message.data}')
        if message.data:
            request = Trigger.Request()
            future = self.horn_client.call_async(request)
            # if future.result() is not None:
                # return future.result().success
            # else:
                # self.get_logger().warn('Horn Service call failed %r' % (future.exception(),))


    def send_request(self):
        request = Trigger.Request()
        future = self.horn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().warn('Horn Service call failed %r' % (future.exception(),))
    

# Check Button Light ON Client 
class ButtonLightOnClient(Node):

    def __init__(self,executor=None):
        super().__init__('button_light_on')
        self.button_light_on_client = self.create_client(Trigger, '/button_light_on')
        self.executor = executor

        while not self.button_light_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Button light on service not available, waiting again...')

    def send_request(self):
        request = Trigger.Request()
        future = self.button_light_on_client.call_async(request)
        rclpy.spin_until_future_complete(self, future,executor=self.executor)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Button light on service call failed %r' % (future.exception(),))

# Check Button Light Off Client 
class ButtonLightOffClient(Node):

    def __init__(self,executor=None):
        super().__init__('button_light_off')
        self.button_light_off_client = self.create_client(Trigger, '/button_light_off')
        self.executor = executor

        while not self.button_light_off_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Button light off service not available, waiting again...')

    def send_request(self):
        request = Trigger.Request()
        future = self.button_light_off_client.call_async(request)
        rclpy.spin_until_future_complete(self, future,executor=self.executor)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Button light off service call failed %r' % (future.exception(),))

# Check Button Reset Client 
class ButtonResetClient(Node):

    def __init__(self,executor=None):
        super().__init__('button_reset')
        self.button_reset_client = self.create_client(Trigger, '/button_reset')
        self.executor = executor

        while not self.button_reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Button reset service not available, waiting again...')

    def send_request(self):
        request = Trigger.Request()
        future = self.button_reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future,executor=self.executor)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Button reset service call failed %r' % (future.exception(),))

class EstopResetClient(Node):
    def __init__(self,executor=None):
        super().__init__('e_stop_reset')
        self.e_stop_reset = self.create_client(Trigger,'/hardware/e_stop_reset')
        self.executor = executor
        
    def send_request(self):
        request = Trigger.Request()
        future = self.e_stop_reset.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Estop reset service call failed %r' % (future.exception(),))


class FactoryNavigator(Node):
    def __init__(self,frame_listener,executor=None):
        super().__init__('FactoryNavigator')
        self.battery_percent = 100.0
        timer_period = 0.1
        self.navStatus = "not_started"
        self.navActive = False 
        self.current_goal = 0
        self.sleep_time = 0.1
        self.rate = self.create_rate(1.0/self.sleep_time)
        self.button_status = False
        self.battery_current = 0.0
        self.power_supply_status = 0
        self.wait_count = 0
        self.parallel_corrections = 0

        self.navigator = BasicNavigator()

        # Node to get the goal positions
        self.set_integer_server = SetGoalServer(self)
        self.set_integer_client = SetGoalClient()

        # self.get_tf2_frame = FrameListener(self)
        self.get_tf2_frame = frame_listener
        self.light_on_client = LightOnClient(executor)
        self.light_off_client = LightOffClient(executor)
        self.horn_client = HornClient(executor)
        # qos_horn = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
        # self.horn_cb_group = MutuallyExclusiveCallbackGroup()
        # self.horn_listener = self.create_subscription(Bool,'/horn_on',callback=self.cb_horn, qos_profile=qos_horn, callback_group=self.horn_cb_group)
        self.button_light_on_client = ButtonLightOnClient(executor)
        self.button_light_off_client = ButtonLightOffClient(executor)
        self.button_reset_client = ButtonResetClient(executor)

        self.publisherCmdVel = self.create_publisher(Twist,'/cmd_vel',1)
        qos_odom = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
        odom_cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_listener = self.create_subscription(Odometry,'/odometry/filtered',callback=self.__odom_cb,qos_profile=qos_odom,callback_group=odom_cb_group)
        self.estop_reset = EstopResetClient(executor)
        #

        

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
        goal_pose.pose.position.x = -1.010
        goal_pose.pose.position.y = -4.981
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.751
        goal_pose.pose.orientation.w = 0.659
        self.goal_poses.append(goal_pose)

        # Goal 2 : Unloading station 2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -5.203
        goal_pose.pose.position.y = -5.013
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.999
        goal_pose.pose.orientation.w = 0.003
        self.goal_poses.append(goal_pose)

        # Goal 3 : Unloading station 3
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -5.122
        goal_pose.pose.position.y = 1.651
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.701
        goal_pose.pose.orientation.w = 0.712
        self.goal_poses.append(goal_pose)
    
        # TODO: EVERYTHING below this is the same location
        # Goal 4 : Unloading station 4
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -1.010
        goal_pose.pose.position.y = -4.981
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.751
        goal_pose.pose.orientation.w = 0.659
        self.goal_poses.append(goal_pose)

        # Goal 5 : Unloading station 5
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.686
        goal_pose.pose.position.y = 6.393
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.021
        goal_pose.pose.orientation.w = 0.999
        self.goal_poses.append(goal_pose)

        # Goal 6 : Unloading station 6
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -5.122
        goal_pose.pose.position.y = 1.651
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.701
        goal_pose.pose.orientation.w = 0.712
        self.goal_poses.append(goal_pose)

        # Goal 7 : Unloading station 7
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.686
        goal_pose.pose.position.y = 6.393
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.021
        goal_pose.pose.orientation.w = 0.999
        self.goal_poses.append(goal_pose)
        
        ## ~~~~~Loading + Charging stations ~~~~~~~~~~~
        ## ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Home: Charging Station + Loading Station -- night mode
        #self.goal_load = PoseStamped()
        #self.goal_load.header.frame_id = 'map'
        #self.goal_load.header.stamp = self.navigator.get_clock().now().to_msg()
        #self.goal_load.pose.position.x = 8.681
        #self.goal_load.pose.position.y = -27.309
        #self.goal_load.pose.position.z = 0.0
        #self.goal_load.pose.orientation.x = 0.0
        #self.goal_load.pose.orientation.y = 0.0
        #self.goal_load.pose.orientation.z =  -0.691
        #self.goal_load.pose.orientation.w = 0.722

        # Home: Charging Station + Loading Station -- day mode
        self.goal_load = PoseStamped()
        self.goal_load.header.frame_id = 'map'
        self.goal_load.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_load.pose.position.x = 0.0
        self.goal_load.pose.position.y = 0.0
        self.goal_load.pose.position.z = 0.0
        self.goal_load.pose.orientation.x = 0.0
        self.goal_load.pose.orientation.y = 0.0
        self.goal_load.pose.orientation.z =  0.0
        self.goal_load.pose.orientation.w =  1.0

        # Batery Full and low thresholds
        self.nBatteryFullThreshold = 0.5
        self.nBatteryLowThreshold = 0.30

        # Used to publish status
        self.prev_time = datetime.now()

        self.timer = self.create_timer(timer_period, self.navigate_to_waypoints_or_charger)
        self.subscription_battery_status = self.create_subscription(
            BatteryState,
            '/battery',
            self.get_battery_status,
            10)

        self.subscription_button_status = self.create_subscription(
            Bool,
            '/numato/button',
            self.get_button_status,
            10)

    # def cb_horn(self,message):
    #     self.get_logger().info(f'Toot toot...{message.data}')
    #     if message.data:
    #         self.horn_client.send_request()


    ################################################################################################
    ### Docking functionality
    def __odom_cb(self,message):
        self.last__odom_message = message
    
    def publish_cmd(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        if (msg.linear.x > 0.2):
            msg.linear.x = 0.2
        elif(msg.linear.x < -0.2):
            msg.linear.x = -0.2

        if (msg.angular.z > 0.15):
            msg.angular.z = 0.15
        elif(msg.angular.z < -0.15):
            msg.angular.z = -0.15

        # print(f"   cmd_vel: [{msg.linear.x:.3f}, {msg.angular.z :.3f}]")
        self.publisherCmdVel.publish(msg)

    def get_odom(self) -> np.ndarray:
        """
        Get the current odom of the robot
        :return : 4x4 homogenous matrix, None if not avail
        """
        try:
            return f_util.get_mat_from_odom_msg(
                self.last__odom_message
            )
        except:
            self.get_logger().error(f"Failed to get odom")
            return None

    def get_centre_of_side_markers(self, offset=0.0) -> Pose2D:
        """
        Get centre tf of both side markers, reference to base_link
        :return: tf of the centre [x, y, yaw]
        """
        left_tf =  self.get_tf2_frame.get_tf('charge_left') 
        right_tf =  self.get_tf2_frame.get_tf('charge_right') 

        if (left_tf is None or right_tf is None):
            return None

        # Offset is normal to the line, direction is outward to the
        # the camera. this is to create a point in front of the goal
        return f_util.get_centre_tf(left_tf, right_tf, offset)
    

    def move_with_odom(self, forward: float,speed=0.02) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """
        # self.set_state(self.dock_state, f"move robot: {forward:.2f} m")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = f_util.apply_2d_transform(_initial_tf, (forward, 0, 0))
        # rate = self.create_rate(0.25)

        # second mat
        while rclpy.ok():
            _curr_tf = self.get_odom()
            if _curr_tf is None:
                return False

            dx, dy, dyaw = f_util.compute_tf_diff(_curr_tf, _goal_tf)
            print(f" current x, y, yaw diff: {dx:.3f} | {dy:.2f} | {dyaw:.2f}")

            if abs(dx) < 0.04:
                self.get_logger().warn("Done with move robot")
                return True

            # This makes sure that the robot is actually moving linearly
            ang_vel = f_util.sat_proportional_filter(
                dyaw, abs_max=0.05, factor=0.2)
            l_vel = f_util.bin_filter(dx, speed)

            self.publish_cmd(linear_vel=l_vel, angular_vel=ang_vel)
            self.rate.sleep()
        exit(0)

    def rotate_with_odom(self, rotate: float,speed=0.05) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        :return : success
        """
        # self.set_state(self.dock_state, f"Turn robot: {rotate:.2f} rad")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = f_util.apply_2d_transform(_initial_tf, (0, 0, rotate))

        while rclpy.ok():

            _curr_tf = self.get_odom()
            if _curr_tf is None:
                return False

            dx, dy, dyaw = f_util.compute_tf_diff(_curr_tf, _goal_tf)
            print(f"current x, y, yaw diff: {dx:.2f} | {dy:.2f} | {dyaw:.2f}")

            if abs(dyaw) < 0.03:
                self.get_logger().warn("Done with rotate robot")
                self.parallel_corrections += 1
                return True

            # publish rotate
            ang_vel = f_util.sat_proportional_filter(
                dyaw,
                abs_min=speed,
                abs_max=0.5)
            self.publish_cmd(angular_vel=ang_vel)
            self.rate.sleep()
        exit(0)
    
    def do_parallel_correction(self, offset: float) -> bool:
        """
        A parallel "parking" correction will be executed if the y offset
        exceeds the allowable threshold. Note this is an open loop operation.
        :return: success
        """
        # self.set_state(DockState.PARALLEL_CORRECTION,
                    #    f"activate parallel correction with {offset:.2f}m")

        return (
            # Step 1: turn -90 degrees respective to odom
            
            self.rotate_with_odom(-math.pi/2,speed=0.1)
            # Step 2: parallel correction respective to odom
            and self.move_with_odom(offset,speed=0.1)
            # and 
            # Step 3: turn 90 degrees respective to odom
            and self.rotate_with_odom(math.pi/2,speed=0.1) # self.navigator.spin(spin_dist=math.pi/2,time_allowance=10)
        )
    
    def do_single_side_marker_rotate(self) -> bool:
        """
        This predock's substate which handles single side marker detection,
        adjust the yaw according to the pose estimation of one single marker
        :return : success
        """
        # self.set_state(DockState.PREDOCK, "Try single marker yaw adjustment")

        right_tf =  self.get_tf2_frame.get_tf('charge_right')
        left_tf = self.get_tf2_frame.get_tf('charge_left')

        if left_tf is not None:
            print(f"Rotate with left marker: {'charge_left'}")
            yaw = f_util.get_2d_pose(left_tf)[2]
            yaw = f_util.flip_yaw(yaw)
            return self.navigator.spin(spin_dist=yaw - math.pi/2, time_allowance=10) #self.rotate_with_odom(yaw - math.pi/2)

        if right_tf is not None:
            print(f"Rotate with right marker: {'charge_right'}")
            yaw = f_util.get_2d_pose(right_tf)[2]
            yaw = f_util.flip_yaw(yaw)
            return self.navigator.spin(spin_dist=yaw - math.pi/2, time_allowance=10) # self.rotate_with_odom(yaw - math.pi/2)

        print("Not detecting two side markers, exit state")
        return False

    def do_predock(self):
        ## initial check if both markers are detected, 
        # Prealignment and distance check
        if self.get_centre_of_side_markers() is None:
            if not self.do_single_side_marker_rotate():
                return False

         # start predock loop
        _pose_list = []
        print("Both Markers are detected, running predock loop")
        while rclpy.ok():
            print('do_predock . . . ')
            centre_tf = self.get_centre_of_side_markers()
            if centre_tf is None:
                print("Not detecting two side markers, exit state")
                return False
            centre_tf = f_util.flip_base_frame(centre_tf)
            yaw = centre_tf[2]
            print(yaw)
            
            # Adjust alignement to the charging station line
            if abs(yaw) < 0.10   :
                for _ in range(10):
                        self.rate.sleep()
                self.publish_cmd()
                marker_to_base_link_tf = f_util.get_2d_inverse(centre_tf)
                if len(_pose_list) < 3:
                    remainings = 3 -  len(_pose_list)
                    print(f"Getting {remainings} more samples for averaging")
                    _pose_list.append(marker_to_base_link_tf)
                    continue
                y_offset = -f_util.avg_2d_poses(_pose_list)[1]
                _pose_list = []
                if abs(y_offset) > 0.35:
                    if not self.do_parallel_correction(y_offset):
                        return False
                    self.publish_cmd()
                    for _ in range(10):
                        self.rate.sleep()
                    continue
                return True
            
            ang_vel = f_util.bin_filter(yaw, 0.05)
            self.publish_cmd(angular_vel=ang_vel)
            self.rate.sleep()
        exit(0)

    def do_last_mile(self):
        # Drive forward until close to charger - using only small charger
        remaining_dis = 0.550
        _dir = 1
        while rclpy.ok():
            print("do_last_mile . . .")
            centre_tf_mat =  self.get_tf2_frame.get_tf('charge_center')
            
            # centre_tf_mat = self.get_tf('charge_center')
            # Final Dock based on odom if centre marker is getting to close (when it is not seen)
            if centre_tf_mat is None:
                print("Not detecting centre marker")
                if remaining_dis < 0.20:
                    print(f"move {remaining_dis}m with odom")
                    return self.move_with_odom(_dir*remaining_dis)
                else:
                    print("exceeded max_last_mile_odom with "
                                 "remaining dis of {remaining_dis}, exit!")
                    return False
            centre_tf = f_util.get_2d_pose(centre_tf_mat)
            centre_tf = f_util.flip_base_frame(centre_tf)
            dis, offset , yaw = centre_tf

            yaw -= math.pi/2
            remaining_dis = - dis - 0.550
            ## If yaw is above threshold back it up to try again.
            if abs(offset) > 0.15:
                self.move_with_odom(-0.5,speed=0.2)
                return False
            
            print(f" Approaching Charger -> d: {dis:.3f}, yaw: {yaw:.2f}, offset: {offset:.2f}"
                  f", remaining dis: {remaining_dis:.3f}")

            if (remaining_dis <= 0):
                print(" ~ STOP!! Reach destination! ~")
                self.publish_cmd()
                return True

            ang_vel = f_util.sat_proportional_filter(
                yaw, abs_min=0.0, abs_max=0.05, factor=0.1) #enforce straight driving
            self.publish_cmd(linear_vel=_dir*0.01, angular_vel=ang_vel)
            self.rate.sleep()
        exit(0)

    def do_steer_dock(self) -> bool:
        transition_dis_with_tol = 0.60
        _dir = 1 # Front positon
        while rclpy.ok():
            print("do_steer_dock . . .")
            centre_tf = self.get_centre_of_side_markers(0.0)
            ## If markers aren't detected
            if centre_tf is None:
                centre_tf_mat =  self.get_tf2_frame.get_tf('charge_center')
                if centre_tf_mat is None:
                    print("Not able to locate centre marker too")
                    return False
                dis, _, _ = f_util.get_2d_pose(centre_tf_mat)
                if (abs(dis) > transition_dis_with_tol):
                    print(f"Centre marker is {dis:.3f} m away, too far, exit")
                    return False
                print(f"Centre marker {dis}m away, transition to last_mile")
                return True
            centre_tf = f_util.flip_base_frame(centre_tf)
            dis, offset, _ = centre_tf

            # transition distance to last mile
            # since we offset with offset_from_charger, This should be 0,
            if (dis + transition_dis_with_tol > 0):
                print("Transition to lastmile state")
                self.publish_cmd()
                return True
            print(f"  DETECTED both side markers!! "
                  f"[d: {dis:.2f}, offset: {offset:.2f}]")
            
            ang_vel = f_util.sat_proportional_filter(
                offset, factor=0.3)
            self.publish_cmd(linear_vel=_dir*0.04, angular_vel= -ang_vel)
            self.rate.sleep() 
        exit(0)


    def attemptDock(self):
        print('Starting Dock')
        if self.do_predock() and self.parallel_corrections < 10 and self.do_steer_dock() and self.do_last_mile():
            print('Docked')
            self.parallel_corrections = 0
            return True
        self.parallel_corrections = 0
        return False


    ########################################


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
        self.battery_percent = msg.percentage
        self.battery_current = msg.current
        self.power_supply_status = msg.power_supply_status
        # self.get_logger().info( ">>> Battery Level at %.0d" % (self.nBatteryLevel))
    
    def get_button_status(self, msg):
        self.button_status = msg.data
        #sel throttle_duration_sec=10)
            
      
    def navigate_to_waypoints_or_charger(self):
        
        now = datetime.now()
        #self.get_logger().info( f">>>status  {self.navStatus}", throttle_duration_sec=10)


        if(self.navStatus == "not_started"):

            # Set the robot's initial pose if necessary
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.position.z = 0.0
            initial_pose.pose.orientation.x = 0.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
            if not self.navigator.initial_pose_received:
                self.estop_reset.send_request()
                self.navigator.setInitialPose(initial_pose)
        
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

            self.light_off_client.send_request()
            self.navStatus = "ready"
      
        elif((self.navStatus == "ready" or (self.navStatus == "navigating" and not self.navActive) ) 
            and self.battery_percent <= self.nBatteryLowThreshold):
            
            # print("Battery low")
            # self.navStatus = "preparing_to_go_to_charger"
            # self.navigator.cancelTask()
            # self.navActive = False
            self.navStatus == "pre_load_station"
            self.get_logger().info(f'Charging, >>> Battery Level at {self.battery_percent}, waiting until {self.nBatteryLowThreshold}', throttle_duration_sec=10)

        # elif(self.navStatus == "cancel"): 

        #     self.navStatus = "canceling_navigation"
        #     self.navigator.cancelTask()
        #     self.navActive = False

        # elif(self.navStatus == "canceling_navigation"):
        #     print('Cancelling nav. before going to charger')
        #     if(self.navigator.isTaskComplete()):
        #         self.navStatus == "ready"  
        #     self.navActive = False

        elif(self.navStatus == "ready"):
            self.get_logger().info(f'Waiting for Command, >>> Battery Level at {self.battery_percent}', throttle_duration_sec=10)
        
            # Call the service server and record the output
            self.current_goal = self.set_integer_server.input_integer
            if self.current_goal > 0 and self.current_goal <= 6:
                self.navigate_to_goal =  self.goal_poses[self.current_goal]
                self.horn_client.send_request()
                self.light_on_client.send_request()
                self.button_reset_client.send_request()
                self.navActive = False
                # self.navigator.clearAllCostmaps()
                if self.battery_current  > 0.01:
                    self.navStatus = "backup"
                else:
                    self.navStatus = "backup"
                    # self.navStatus = "navigating" # In case the charger is not on, we backup anyway
                     
            # Navigate to Charging station if started at some other place
            elif self.current_goal == 7: #TODO - -May need to hcnage this number
                self.horn_client.send_request()
                self.light_on_client.send_request()
                self.button_reset_client.send_request()
                if self.battery_current  > 0.01:
                    self.navStatus = "backup"
                else:
                    self.navStatus = "pre_load_station"                  

        elif(self.navStatus == "backup"):
            output = self.move_with_odom(forward=-0.75,speed=0.2) # self.navigator.backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10)
            if output:
                self.navigator.goToPose(self.navigate_to_goal)
                self.navActive = False
                if self.current_goal == 7:
                    self.navStatus = "pre_load_station" 
                else:
                    self.navStatus = "navigating"
 

        elif(self.navStatus == "navigating"):
            self.get_logger().info(f'>>> Navigating to the goal {self.current_goal}', throttle_duration_sec=10)
            if(not self.navigator.isTaskComplete()):
                self.navActive = True
            else:
                self.button_light_on_client.send_request()
                self.button_reset_client.send_request()
                self.navStatus = "unload"
                self.navActive = False

        elif(self.navStatus == "unload"):
            self.get_logger().info(f'>>> Waiting to Unload at Station {self.current_goal}, waited for {self.wait_count/10.0}', throttle_duration_sec=10)
            self.wait_count += 1      
            self.navActive = False
            if self.button_status or self.wait_count > 600:
                self.wait_count = 0
                self.button_light_off_client.send_request()
                self.button_reset_client.send_request()
                self.horn_client.send_request()
                self.navStatus = "pre_load_station"

        elif(self.navStatus == "pre_load_station"):
            if(self.navigator.isTaskComplete()):
                self.navStatus = "load"
                self.navigator.goToPose(self.goal_load)   
            self.navActive = False

        elif(self.navStatus == "load"):
            self.get_logger().info(">>> Navigating to Loading Station" , throttle_duration_sec=10)
            if(self.navigator.isTaskComplete()):
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.parallel_corrections = 0
                    self.navStatus = "docking"
                else:
                    self.navStatus = "pre_load_station"
            self.navActive = False

        elif(self.navStatus == "docking"):
            self.get_logger().info(">>> Docking ..." , throttle_duration_sec=1 )
            self.navStatus = "docking"
            self.navActive = False  
            output = self.attemptDock()
            if output:
                self.navStatus = "completed"
            else:
                self.navStatus = "pre_load_station"

        elif(self.navStatus == "completed"):

            self.set_integer_client.set_integers(0)
            self.current_goal = self.set_integer_server.input_integer
            self.light_off_client.send_request()

            result = self.navigator.getResult()
            if self.current_goal ==0: 
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
         

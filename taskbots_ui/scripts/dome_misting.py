#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy

from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
import subprocess


class MistingLevelNode(Node):
    def __init__(self):
        super().__init__("misting_level")
        self.misting_pub_ = self.create_publisher(Int16,"misting_level",10)

def main():
    rclpy.init()
    misting_pub_node = MistingLevelNode()
    # rclpy.spin(misting_pub_node)

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.


    circle_center = (2.5,6.42)
    angle_offset = -3.14
    dome_radius = 1.85 # Actual dome is 2.67 - 0.42 width
    no_pts = 20

    internal_route = []
    for i in range(no_pts):
        pos_angle = 360.0/no_pts*i
        point = [circle_center[0] + dome_radius*math.cos(angle_offset+math.radians(pos_angle)),
                 circle_center[1] + dome_radius*math.sin(angle_offset+math.radians(pos_angle)),
                 math.sin((angle_offset+math.radians(90+pos_angle))/2.0),
                 math.cos((angle_offset+math.radians(90+pos_angle))/2.0)]
        internal_route.append(point)

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = circle_center[0] 
    initial_pose.pose.position.y = circle_center[1]
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass
    


   
    while rclpy.ok():
        # Send our route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        #inspection_pose.pose.orientation.z = 1.0
        #inspection_pose.pose.orientation.w = 0.0
        for pt in internal_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_pose.pose.orientation.z = pt[2]
            inspection_pose.pose.orientation.w = pt[3]
            inspection_points.append(deepcopy(inspection_pose))
        print(f"Pose {inspection_points[-1].pose.position.y}, {internal_route[-1][1]}")
        navigator.goThroughPoses(inspection_points)
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 200 == 0:
                misting_level_msg = Int16()
                misting_level_msg.data = 3
                misting_pub_node.misting_pub_.publish(misting_level_msg)
                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=180.0
                ):
                    print('Navigation has exceeded timeout of 180s, canceling request.')
                    navigator.cancelTask()
        result = navigator.getResult()
        if navigator.getResult() == TaskResult.SUCCEEDED:
            #navigator.cancelTask()
            last_point = internal_route.pop()
            internal_route.insert(0,last_point)
    misting_level_msg = Int16()
    misting_level_msg.data = 0
    misting_pub_node.misting_pub_.publish(misting_level_msg)
    exit(0)


if __name__ == '__main__':
    main()

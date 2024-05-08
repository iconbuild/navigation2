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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""

def route(navigator,inspection_points):
    navigator.followWaypoints(inspection_points)
    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))
    result = navigator.getResult()
    return result


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    inspection_route = [
        [159.85,-130.96, 0.85,0.52],
        [165.21, -137.20, -0.47,0.87],
        [163.98, -132.97, 0.29,0.95],
        [160.33, -137.74, -0.94,0.33]]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = inspection_route[0][0] 
    initial_pose.pose.position.y = inspection_route[0][1]
    initial_pose.pose.orientation.z = inspection_route[0][2]
    initial_pose.pose.orientation.w = inspection_route[0][3]
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send our route
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    #inspection_pose.pose.orientation.z = 1.0
    #inspection_pose.pose.orientation.w = 0.0
    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_pose.pose.orientation.z = pt[2]
        inspection_pose.pose.orientation.w = pt[3]
        inspection_points.append(deepcopy(inspection_pose))
    
    result = route(navigator,inspection_points)
    while result!=TaskResult.CANCELED or result!=TaskResult.FAILED:
        result=route(navigator,inspection_points)
    exit(0)


if __name__ == '__main__':
    main()

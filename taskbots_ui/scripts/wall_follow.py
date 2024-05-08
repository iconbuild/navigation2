from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration


"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route = [[-1.0,2.96],\
        [-1.0,6.02],
        [-1.22,13.8],
        [-1.35,20.3],
        [-1.29,27.3],
        [-1.3,29.0],
        [3.45,29.0],
        [3.66,26.7],
        [2.6,26.0],
        [3.53,24.5],
        [2.73,24.0],
        [1.82,25.0],
        [1.76,26.6],
        [-0.05,26.0],
        [0.459,24.2],
        [-1.38,26.0],
        [-0.237,22.3],
        [2.86,22.2],
        [3.36,21.3],
        [1.96,20.3],
        [0.465,20.7],
        [-1.27,20.6],
        [-0.277,19.1],
        [2.09,19.0],
        [3.49,18.1],
        [2.1,17.1],
        [0.201,17.5],
        [-1.29,17.4],
        [0.7,15.9],
        [0.205,13.9],
        [2.43,13.9],
        [3.27,15.5],
        [4.22,14.0],
        [7.02,14.0],
        [7.47,13.3],
        [7.41,10.5],
        [7.5,8.81],
        [7.42,6.0],
        [6.39,5.82],
        [6.51,3.97],
        [7.51,3.61],
        [5.56,3.15],
        [4.43,3.83],
        [4.55,9.43],
        [3.41,11.2],
        [0.271,11.2],
        [-0.761,11.8],
        [-0.325,9.73],
        [2.81,9.75],
        [3.12,7.11],
        [2.9,3.98],
        [3.21,2.1]]

    # Set our demo's initial pose
    security_route.reverse()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.5470
    initial_pose.pose.position.y = 4.228
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.033
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Do security route until dead
    while rclpy.ok():
        # Send our route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        navigator.goThroughPoses(route_poses)

        # Do something during our route (e.x. AI detection on camera images for anomalies)
        # Simply print ETA for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180000.0):
                    print('Navigation has exceeded timeout of 180s, canceling request.')
                    navigator.cancelTask()

        # If at end of route, reverse the route to restart
        # security_route.reverse()

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Security route failed! Restarting from other side...')

    exit(0)


if __name__ == '__main__':
    main()

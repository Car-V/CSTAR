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

import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult#, NavigationResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    inspection_route = []
    length = 5.0 # update
    width = 1.0
    step = 1.0
    y = 0.0
    while y <= width:
        # Forward direction along the y-axis
        x = 0.0
        while x <= length:
            inspection_route.append((round(x, 2), round(y, 2)))
            x += step

        y += step
        if y > width:
            break

        # Reverse direction along the y-axis
        x = length
        while x >= 0.0:
            inspection_route.append((round(x, 2), round(y, 2)))
            x -= step

        y += step

    # Example usage
    # inspection_route = [
    #     [5.0, 0.0],
    #     [-5.0, -5.0],
    #     [-5.0, 5.0]
    # ]

    # out
    # Set our demo's initial pose # if we already have it set then don't need this, keep for now
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should waitUntilNav2Active() instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    while rclpy.ok():
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.orientation.z = 1.0
        inspection_pose.pose.orientation.w = 0.0
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_points.append(deepcopy(inspection_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)) + ' ' + str(inspection_points[feedback.current_waypoint]))            

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Inspection complete! Returning to start')
        elif result == TaskResult.CANCELED:
            print('Inspection canceled! Returning to start')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Inspection failed! Retrying first waypoint')
            navigator.goToPose(inspection_points[0])

        while not navigator.isTaskComplete():
            pass


    # navigator.lifecycleShutdown()

    # exit(0)

    # set our demo's goal poses to follow
#     goal_poses = []
#     goal_pose1 = PoseStamped()
#     goal_pose1.header.frame_id = 'map'
#     goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose1.pose.position.x = 10.15
#     goal_pose1.pose.position.y = -0.77
#     goal_pose1.pose.orientation.w = 1.0
#     goal_pose1.pose.orientation.z = 0.0
#     goal_poses.append(goal_pose1)
# # additional goals can be appended
#     goal_pose2 = PoseStamped()
#     goal_pose2.header.frame_id = 'map'
#     goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose2.pose.position.x = 17.86
#     goal_pose2.pose.position.y = -0.77
#     goal_pose2.pose.orientation.w = 1.0
#     goal_pose2.pose.orientation.z = 0.0
#     goal_poses.append(goal_pose2)
#     goal_pose3 = PoseStamped()
#     goal_pose3.header.frame_id = 'map'
#     goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose3.pose.position.x = 21.58
#     goal_pose3.pose.position.y = -3.5
#     goal_pose3.pose.orientation.w = 1.0
#     goal_pose3.pose.orientation.z = 0.0
#     goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    # nav_start = navigator.get_clock().now()
    # navigator.followWaypoints(goal_poses)

    # i = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print(
    #             'Executing current waypoint: '
    #             + str(feedback.current_waypoint + 1)
    #             + '/'
    #             + str(len(goal_poses))
    #         )
    #         now = navigator.get_clock().now()

    #         # Some navigation timeout to demo cancellation
    #         if now - nav_start > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some follow waypoints request change to demo preemption
    #         if now - nav_start > Duration(seconds=35.0):
    #             goal_pose4 = PoseStamped()
    #             goal_pose4.header.frame_id = 'map'
    #             goal_pose4.header.stamp = now.to_msg()
    #             goal_pose4.pose.position.x = 0.0
    #             goal_pose4.pose.position.y = 0.0
    #             goal_pose4.pose.orientation.w = 1.0
    #             goal_pose4.pose.orientation.z = 0.0
    #             goal_poses = [goal_pose4]
    #             nav_start = now
    #             navigator.followWaypoints(goal_poses)

    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     (error_code, error_msg) = navigator.getTaskError()
    #     print('Goal failed!{error_code}:{error_msg}')
    # else:
    #     print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    # exit(0)


if __name__ == '__main__':
    main()

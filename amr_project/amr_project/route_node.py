#!/usr/bin/env python3
"""
 Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Licensed under the Apache License, Version 2.0 (the "License").
 You may not use this file except in compliance with the License.
 A copy of the License is located at

  http://aws.amazon.com/apache2.0

 or in the "license" file accompanying this file. This file is distributed
 on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 express or implied. See the License for the specific language governing
 permissions and limitations under the License.
"""

import itertools
import random
import yaml
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import FollowPath
import os
from std_msgs.msg import Bool


class RouteManager(Node):
    """
    Send goals to the navigation2 stack for the specified route. Routes forever.

   Loads the route from yaml.
   Use RViz to record 2D nav goals.
   Echo the input goal on topic /move_base_simple/goal

   Format:

        mode: inorder
        poses:
            - pose:
                  position:
                    x: -5.41667556763
                    y: -3.14395284653
                    z: 0.0
                  orientation:
                    x: 0.0
                    y: 0.0
                    z: 0.785181432231
                    w: 0.619265789851

    """


    # Return an iterator over the goals
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random': lambda goals: (random.shuffle(goals), itertools.cycle(goals))[1],
    }

    def __init__(self):
        super().__init__('route_manager', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        # initialize the route
        self.route = []
        # initialize the goal
        self.current_goal = NavigateToPose.Goal()
        self.BoolSubscriber = self.create_subscription(Bool, 'localization_complete', self.bool_callback, 10)
        # initialize the flag for localization to check if localization is complete or not
        self.localization_complete = False
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()

        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.follow_path_client.wait_for_server()

        route_file_info = self.get_parameter('route_file').value
        route_file_path = os.path.join(os.getcwd(), 'src', 'amr_project', 'amr_project', 'routes.yaml')
        # Read the route file and initialize the goals
        with open(route_file_path, 'r') as f:
            route_file_contents = f.read()
        route_yaml = yaml.safe_load(route_file_contents)

        self.route_mode = route_yaml['mode']

        if self.route_mode not in RouteManager.route_modes:
            self.get_logger().error(
                "Route mode '%s' unknown, exiting route manager" % (self.route_mode,))
            return

        poses = route_yaml['poses']
        if not poses:
            self.get_logger().info("Route manager initialized no goals, unable to route")

        self.goals = RouteManager.route_modes[self.route_mode](poses)
        self.number_of_goals = 0
        self.length = len(poses)
        self.get_logger().info(
            "Route manager initialized with %s goals in %s mode" % (len(poses), self.route_mode,))

    def to_move_goal(self, pose):
        '''
        Take a pose and return a NavigateToPose goal
        -Input: pose
        -Output: goal
        '''
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position = Point(**pose['pose']['position'])
        goal.pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def goal_response_callback(self, future):
        '''
        Callback for when a goal is sent
        -Input: future
        -Output: None
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def route_forever(self):
        '''
        Send the next goal to the navigation stack
        '''
        self.get_logger().info("Route mode is '%s', getting next goal" % (self.route_mode,))
        try:
            current_goal = self.to_move_goal(next(self.goals))
        except StopIteration:
            self.get_logger().info("All goals have been visited, stopping route manager.")
            return
        self.number_of_goals += 1
        self.get_logger().info("Sending target goal: (%s, %s)" % (current_goal.pose.pose.position.x, current_goal.pose.pose.position.y))
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(
            current_goal,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_result_callback(self, future: NavigateToPose.Result):
        '''
        Callback for when a goal is completed
        -Input: future
        -Output: None
        '''
        result = future.result().result
        # Expecting empty result (std_msgs::Empty) for NavigateToPose
        self.get_logger().info('Result: {0}'.format(result.result))
        if self.number_of_goals < self.length:
            self.route_forever()
        else:
            self.get_logger().info("No more goals, stopping route manager.")

    def feedback_callback(self, feedback_msg: NavigateToPose.Feedback):
        # NavigateToPose should have no feedback
        # self.get_logger().info('Received feedback')
        pass
    
    def bool_callback(self, msg):
      # Callback function for the subscriber to the localization_complete topic to check if localization is complete or not
      self.localization_complete = msg.data
      
def main():
    rclpy.init()
    try:
        route_manager = RouteManager()
        count=0
        # Wait for localization to complete before starting the route manager
        while route_manager.localization_complete == False:
            if count==0:
                route_manager.get_logger().info("Waiting for localization to complete")
                count=1
            rclpy.spin_once(route_manager)
        route_manager.get_logger().info("Localization complete, starting route manager")
        route_manager.route_forever()

        rclpy.spin(route_manager)
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in route_manager:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

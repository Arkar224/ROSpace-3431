#!/usr/bin/env python3

import csv, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.declare_parameter('waypoints_file', '/home/troublemaker/comp3431/turtlebot_ws/landmarks.csv')
        self.csv_path = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.navigator = BasicNavigator()
        self.start_pose = self.make_pose(0.0, 0.0, yaw_d=0.0)
        self.back_pose = self.make_pose(0.0, 0.0, yaw_d=180.0)
        self.wpts = []

    def make_pose(self, x, y, yaw_d=0.0, frame='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x); pose.pose.position.y = float(y)
        yaw = math.radians(yaw_d)
        pose.pose.orientation.z = math.sin(yaw/2.0)
        pose.pose.orientation.w = math.cos(yaw/2.0)
        return pose

    def load_csv(self):
        self.wpts.clear()
        with open(self.csv_path, 'r') as f:
            for row in csv.reader(f):
                if len(row) >= 2:
                    x, y = map(float, row[:2])
                    self.wpts.append(self.make_pose(x, y))

    def run(self):
        self.navigator.waitUntilNav2Active()
        self.load_csv()
        if not self.wpts:
            self.get_logger().warn("No waypoints loaded.")
            return
        
        # go through all waypoints
        for i, wp in enumerate(self.wpts, start=1):
            self.get_logger().info(f'going to waypoint {i}/{len(self.wpts)}')
            self.navigator.goToPose(wp)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            res = self.navigator.getResult()
            if res == TaskResult.SUCCEEDED:
                self.get_logger().info(f'reached waypoint {i}')
                time.sleep(1.5)
            else:
                self.get_logger().info(f'failed to reach waypoint {i} .... kill the job')
                return
    
        # return to start point
        self.get_logger().info("returning to start points after exploring all")
        self.navigator.goToPose(self.back_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("yeah all done safely")
        else:
            self.get_logger().info("oops something's happening")

def main():
    rclpy.init()
    node = WaypointNavigator()
    node.navigator.setInitialPose(node.start_pose)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

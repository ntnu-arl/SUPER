#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import tf

class WaypointPublisher:
    def __init__(self):
        # Define waypoints (x, y, z)
        self.waypoints = [
            (0.0, 0.0, 1.0),
            (10.0, 0.0, 1.0),
            (10.0, -10.0, 1.0),
            (0.0, -10.0, 1.0)
        ]
        self.current_idx = 0
        self.threshold = 5.0  # meters

        # Publisher for goals
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

        # Subscriber to odometry
        rospy.Subscriber("/firefly/ground_truth/odometry_throttled_100", Odometry, self.odom_callback)

        self.first = True
        self.robot_pose = None  # store latest odometry pose

        # # Publish the first goal
        # self.publish_goal()

    def publish_goal(self):
        if self.current_idx < len(self.waypoints):
            x, y, z = self.waypoints[self.current_idx]
            # Determine orientation (yaw) towards previous point
            if self.current_idx == 0:
                # First waypoint: direction from robot to first goal
                if self.robot_pose is None:
                    rospy.logwarn("No odometry yet, cannot publish first goal.")
                    return
                prev_x = self.robot_pose[0]
                prev_y = self.robot_pose[1]
            else:
                prev_x, prev_y, _ = self.waypoints[self.current_idx - 1]

            dx = prev_x - x
            dy = prev_y - y
            yaw = math.atan2(dy, dx)

            # Convert yaw to quaternion
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "world"  # or "odom", depending on your setup
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = z
            goal_msg.pose.orientation = Quaternion(*quat)
            rospy.sleep(0.2)
            for i in range(5):
                self.goal_pub.publish(goal_msg)
                rospy.sleep(0.1)  # Ensure message is sent
            rospy.loginfo("Published goal %d: (%.2f, %.2f, %.2f)", self.current_idx, x, y, z)
        else:
            rospy.loginfo("All waypoints completed.")

    def odom_callback(self, msg):
        if self.current_idx >= len(self.waypoints):
            return

        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        if self.first:
            self.first = False
            self.publish_goal()

        # Current robot position
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        rz = msg.pose.pose.position.z

        # Current goal
        gx, gy, gz = self.waypoints[self.current_idx]

        # Distance to goal
        dist = math.sqrt((rx - gx)**2 + (ry - gy)**2 + (rz - gz)**2)

        if dist < self.threshold:
            rospy.loginfo("Reached waypoint %d", self.current_idx)
            self.current_idx += 1
            self.publish_goal()

if __name__ == "__main__":
    rospy.init_node("waypoint_publisher")
    wp = WaypointPublisher()
    rospy.spin()

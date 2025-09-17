#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathToGoal:
    def __init__(self):
        # Publisher for the goal
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

        # Subscriber to the path
        rospy.Subscriber("/gbplanner_path", Path, self.path_callback)

    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("Received empty path, skipping...")
            return

        # Take the last pose in the path
        last_pose = msg.poses[-1]

        # Publish it as the goal
        self.goal_pub.publish(last_pose)

        rospy.loginfo_throttle(5, "Published last pose from path as /goal")

if __name__ == "__main__":
    rospy.init_node("path_to_goal", anonymous=True)
    PathToGoal()
    rospy.spin()

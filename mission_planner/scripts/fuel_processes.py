#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Transform, Quaternion, Twist, Point
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from quadrotor_msgs.msg import PositionCommand


class OdomToPoseStamped:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/rmf_obelix/ground_truth/odometry', Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher('/rmf_obelix/fake_pose', PoseStamped, queue_size=1)
    
    def odom_callback(self, odom_msg):
        # Create a PoseStamped message
        pose_stamped_msg = PoseStamped()

        # Set the header (frame_id and timestamp)
        pose_stamped_msg.header.frame_id = odom_msg.header.frame_id
        pose_stamped_msg.header.stamp = odom_msg.header.stamp

        # Set the pose (position and orientation)
        pose_stamped_msg.pose = odom_msg.pose.pose

        # Publish the PoseStamped message
        self.pose_pub.publish(pose_stamped_msg)

class PoseToModelState:
    def __init__(self):

        # Subscriber to the PoseStamped topic
        self.pose_sub = rospy.Subscriber('/odom_visualization/pose', PoseStamped, self.pose_callback)
        self.pose_sub = rospy.Subscriber('/lidar_slam/odom', Odometry, self.odom_callback)

        # Publisher to the gazebo_msgs/ModelState topic
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        # The name of the model you want to control
        self.model_name = "rmf_obelix"

    def pose_callback(self, pose_msg):
        # Create a ModelState message
        model_state_msg = ModelState()

        # Set the model name
        model_state_msg.model_name = self.model_name

        # Set the pose from the PoseStamped message
        model_state_msg.pose = pose_msg.pose

        # Set the twist to zero (no movement)
        model_state_msg.twist.linear.x = 0
        model_state_msg.twist.linear.y = 0
        model_state_msg.twist.linear.z = 0
        model_state_msg.twist.angular.x = 0
        model_state_msg.twist.angular.y = 0
        model_state_msg.twist.angular.z = 0

        # Set the reference frame (empty means world)
        model_state_msg.reference_frame = 'world'

        # Publish the ModelState message to the gazebo topic
        self.model_state_pub.publish(model_state_msg)
    
    def odom_callback(self, odom_msg):
        # Create a ModelState message
        model_state_msg = ModelState()

        # Set the model name
        model_state_msg.model_name = self.model_name

        # Set the pose from the Odometry message
        model_state_msg.pose = odom_msg.pose.pose

        # Set the twist to zero (no movement)
        model_state_msg.twist = odom_msg.twist.twist
        # model_state_msg.twist.linear.x = 0
        # model_state_msg.twist.linear.y = 0
        # model_state_msg.twist.linear.z = 0
        # model_state_msg.twist.angular.x = 0
        # model_state_msg.twist.angular.y = 0
        # model_state_msg.twist.angular.z = 0

        # Set the reference frame (empty means world)
        model_state_msg.reference_frame = 'world'

        # Publish the ModelState message to the gazebo topic
        self.model_state_pub.publish(model_state_msg)

    def run(self):
        rospy.spin()

class PosCMDToTrajCMD:
    def __init__(self):
        self.traj_pub = rospy.Publisher('/rmf_obelix/command/trajectory', MultiDOFJointTrajectory, queue_size=1)
        self.pos_cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.pos_callback)
    
    def pos_callback(self, pos_cmd):
        traj = MultiDOFJointTrajectory()
        traj_pt = MultiDOFJointTrajectoryPoint()

        quat_sp = Rotation.from_euler('zyx', [pos_cmd.yaw, 0., 0.]).as_quat()
        p = Transform(translation=pos_cmd.position, rotation=Quaternion(quat_sp[0], quat_sp[1], quat_sp[2], quat_sp[3]))
        traj_pt.transforms.append(p)

        # v = Twist()
        # v.linear = pos_cmd.velocity
        # v.angular.z = pos_cmd.yaw_dot
        # traj_pt.velocities.append(v)

        # a = Twist()
        # a.linear = pos_cmd.acceleration
        # traj_pt.accelerations.append(a)

        traj_pt.time_from_start = rospy.Duration(0, 2e7)

        traj.points.append(traj_pt)
        traj.header = pos_cmd.header
        
        self.traj_pub.publish(traj)

class cmdToOdom:
    def __init__(self):
        self.odom_pub = rospy.Publisher('/cmd_odom', Odometry, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.pos_callback)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

        self.last_odom = Odometry()
        self.odom_ready = False

    def pos_callback(self, pos_cmd=PositionCommand):
        cmd_odom = Odometry()
        cmd_odom.header = pos_cmd.header
        cmd_odom.pose.pose.position = pos_cmd.position

        quat_sp = Rotation.from_euler('zyx', [pos_cmd.yaw, 0., 0.]).as_quat()
        cmd_odom.pose.pose.orientation.x = quat_sp[0]
        cmd_odom.pose.pose.orientation.y = quat_sp[1]
        cmd_odom.pose.pose.orientation.z = quat_sp[2]
        cmd_odom.pose.pose.orientation.w = quat_sp[3]

        cmd_odom.twist.twist.linear = pos_cmd.velocity
        cmd_odom.twist.twist.angular.z = pos_cmd.yaw_dot

        # self.odom_pub.publish(cmd_odom)
        self.last_odom = cmd_odom
        self.odom_ready = True
    
    def timer_callback(self, timer):
        if self.odom_ready:
            self.odom_pub.publish(self.last_odom)

if __name__ == '__main__':
    rospy.init_node('pose_to_model_state_node')
    try:
        pose_to_model_state = PoseToModelState()
        # odom_to_ps = OdomToPoseStamped()
        # pos_cmd_to_traj = PosCMDToTrajCMD()
        # pos_cmd_to_odom = cmdToOdom()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

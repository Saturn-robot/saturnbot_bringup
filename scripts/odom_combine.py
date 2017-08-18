#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster
from tf.transformations import *

"""
Combine data from imu and wheels' odometry,
and publish accurate odometry and TF between odom and base_frame
"""
class OdomCombine:
    def __init__(self, base_frame='base_footprint', name='odom_combine'):
        self.base_frame = base_frame
        self.name = name
        rospy.init_node(name, log_level=rospy.INFO)
        rospy.Subscriber('arduino/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        self.odom_posit = Point()
        self.odom_orient = Quaternion()
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def imu_callback(self, imu_data):
        rospy.logdebug("Get imu data!")
        # Acquire imu data from imu device
        self.odom_orient = imu_data.orientation



    def odom_callback(self, odom_data):
        rospy.logdebug("Get odometry data!")
        # Acquire data from odometry data from wheels' encoder
        self.odom_posit = odom_data.pose.pose.position
        self.linear_vel = odom_data.twist.twist.linear.x
        self.angular_vel = odom_data.twist.twist.angular.z

    def odom_combine(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            #(r, p, y) = euler_from_quaternion((self.odom_orient.x, self.odom_orient.y,
            #                                   self.odom_orient.z, self.odom_orient.w))
            #(x, y, z, w) = quaternion_from_euler(y, p, r)
            # Create the odometry transform frame broadcaster.
            z = self.odom_orient.z
            w = self.odom_orient.w
            self.odomBroadcaster.sendTransform((self.odom_posit.x, self.odom_posit.y, self.odom_posit.z),
                                               (0, 0, z, w),
                                               now,
                                               self.base_frame,
                                               "odom")
            rospy.loginfo("Orient Data: %s %s", self.odom_orient.z, self.odom_orient.w)
            # Create odom data and publish it
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position = self.odom_posit
            odom.pose.pose.orientation = self.odom_orient
            odom.twist.twist.linear.x = self.linear_vel
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.angular_vel

            self.odomPub.publish(odom)
            rate.sleep()

if __name__ == "__main__":
    odomc = OdomCombine()
    odomc.odom_combine()

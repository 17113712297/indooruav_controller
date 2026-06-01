#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import (
    quaternion_from_euler,
    quaternion_multiply,
    euler_from_quaternion,
    quaternion_matrix
)

# ===================== 可修改参数 =====================

ODOMETRY_ROTATE_PITCH_ANGLE_IN_DEG = 24.1
ODOMETRY_ROTATE_YAW_ANGLE_IN_DEG = -2.8
INPUT_ODOMETRY_TOPIC = "/Odometry"
OUTPUT_ROTATED_ODOMETRY_TOPIC = "/Odometry_rotate"

INPUT_GLOBAL_ODOMETRY_TOPIC = "/Odometry_global"
OUTPUT_ROTATED_GLOBAL_ODOMETRY_TOPIC = "/Odometry_global_rotate"

# ======================================================


class OdometryFrameRotator(object):
    def __init__(self):
        self.rotate_pitch_rad = math.radians(ODOMETRY_ROTATE_PITCH_ANGLE_IN_DEG)
        self.rotate_yaw_rad = math.radians(ODOMETRY_ROTATE_YAW_ANGLE_IN_DEG)
        self.rotate_roll_rad = 0.0

        self.rotation_quaternion = quaternion_from_euler(
            self.rotate_roll_rad,
            self.rotate_pitch_rad,
            self.rotate_yaw_rad
        )

        self.rotation_matrix = quaternion_matrix(self.rotation_quaternion)

        self.rotated_odom_pub = rospy.Publisher(
            OUTPUT_ROTATED_ODOMETRY_TOPIC,
            Odometry,
            queue_size=10
        )

        self.rotated_global_odom_pub = rospy.Publisher(
            OUTPUT_ROTATED_GLOBAL_ODOMETRY_TOPIC,
            Odometry,
            queue_size=10
        )

        self.odom_sub = rospy.Subscriber(
            INPUT_ODOMETRY_TOPIC,
            Odometry,
            self.odom_callback,
            queue_size=10
        )

        self.global_odom_sub = rospy.Subscriber(
            INPUT_GLOBAL_ODOMETRY_TOPIC,
            Odometry,
            self.global_odom_callback,
            queue_size=10
        )

        rospy.loginfo(
            "odometry_frame_rotator started. pitch: %.2f deg, yaw: %.2f deg",
            ODOMETRY_ROTATE_PITCH_ANGLE_IN_DEG,
            ODOMETRY_ROTATE_YAW_ANGLE_IN_DEG
        )

    def odom_callback(self, msg):
        rotated_msg = self.rotate_odometry_frame(msg)
        rotated_msg.header.frame_id = "odom_rotate"
        self.rotated_odom_pub.publish(rotated_msg)

    def global_odom_callback(self, msg):
        rotated_msg = self.rotate_odometry_frame(msg)
        rotated_msg.header.frame_id = "odom_global_rotate"
        self.rotated_global_odom_pub.publish(rotated_msg)

    def rotate_odometry_frame(self, msg):
        position_vector = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            1.0
        ])

        rotated_position = np.dot(self.rotation_matrix, position_vector)

        original_quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        rotated_quaternion = quaternion_multiply(
            self.rotation_quaternion,
            original_quaternion
        )

        roll, pitch, yaw = euler_from_quaternion(rotated_quaternion)

        pitch -= self.rotate_pitch_rad
        yaw -= self.rotate_yaw_rad

        final_quaternion = quaternion_from_euler(roll, pitch, yaw)

        rotated_odom_msg = Odometry()

        rotated_odom_msg.header.stamp = msg.header.stamp
        rotated_odom_msg.header.frame_id = msg.header.frame_id
        rotated_odom_msg.child_frame_id = msg.child_frame_id

        rotated_odom_msg.pose.pose.position.x = rotated_position[0]
        rotated_odom_msg.pose.pose.position.y = rotated_position[1]
        rotated_odom_msg.pose.pose.position.z = rotated_position[2]

        rotated_odom_msg.pose.pose.orientation.x = final_quaternion[0]
        rotated_odom_msg.pose.pose.orientation.y = final_quaternion[1]
        rotated_odom_msg.pose.pose.orientation.z = final_quaternion[2]
        rotated_odom_msg.pose.pose.orientation.w = final_quaternion[3]

        rotated_odom_msg.pose.covariance = msg.pose.covariance
        rotated_odom_msg.twist = msg.twist

        return rotated_odom_msg


def main():
    rospy.init_node("odometry_frame_rotator", anonymous=False)
    OdometryFrameRotator()
    rospy.spin()


if __name__ == "__main__":
    main()

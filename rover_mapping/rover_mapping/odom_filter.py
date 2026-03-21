#!/usr/bin/env python3
"""
odom_filter.py — Odometry noise filter for rf2o laser odometry

rf2o generates small phantom motions from scan-to-scan laser noise even when
the robot is completely stationary. This node filters those out by zeroing
velocities below a configurable threshold before broadcasting the TF that
slam_toolbox uses for pose estimation.

Also fills in proper covariance matrices since rf2o outputs all zeros,
which confuses slam_toolbox's trust weighting.

Subscribes:  /odom_rf2o      (nav_msgs/Odometry) — raw rf2o output
Publishes:   /odom_filtered  (nav_msgs/Odometry) — cleaned odometry
Broadcasts:  odom -> base_footprint TF

Setup:
  - rf2o must have publish_tf: false (this node takes over TF broadcasting)
  - rover_params.yaml must have pub_odom_tf: false

Tuning:
  - linear_threshold:  increase if map still twitches while stationary
  - angular_threshold: increase if map still rotates while stationary
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdomFilter(Node):

    def __init__(self):
        super().__init__('odom_filter')

        # -- Parameters --------------------------------------------------------
        self.declare_parameter('linear_threshold',  0.02)   # m/s
        self.declare_parameter('angular_threshold', 0.02)   # rad/s
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('base_frame',        'base_footprint')

        self.linear_threshold  = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.odom_frame        = self.get_parameter('odom_frame').value
        self.base_frame        = self.get_parameter('base_frame').value

        # Default covariances — rf2o sends all zeros which confuses slam_toolbox.
        # Diagonal order: [x, y, z, roll, pitch, yaw]
        # z/roll/pitch are unused for 2D so set to 1e6 (ignore).
        self.POSE_COV_MOVING = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-2,
        ]
        self.POSE_COV_STOPPED = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-9, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9,
        ]
        self.TWIST_COV_MOVING = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-2,
        ]
        self.TWIST_COV_STOPPED = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-9, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9,
        ]

        self.get_logger().info('OdomFilter started')
        self.get_logger().info(f'  Linear threshold:  {self.linear_threshold} m/s')
        self.get_logger().info(f'  Angular threshold: {self.angular_threshold} rad/s')
        self.get_logger().info(f'  TF: {self.odom_frame} -> {self.base_frame}')

        # -- State -------------------------------------------------------------
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # -- TF Broadcaster ----------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # -- Publisher ---------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '/odom_filtered', 10)

        # -- Subscriber --------------------------------------------------------
        self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self._odom_callback,
            10
        )

    def _odom_callback(self, msg: Odometry):
        """
        Receive rf2o odometry, apply noise threshold, update pose,
        publish filtered odometry and broadcast TF.
        """
        vx = msg.twist.twist.linear.x
        vw = msg.twist.twist.angular.z

        # Apply deadzone
        if abs(vx) < self.linear_threshold:
            vx = 0.0
        if abs(vw) < self.angular_threshold:
            vw = 0.0

        moving = (vx != 0.0 or vw != 0.0)

        # Only update stored pose when motion clears the threshold
        if moving:
            self.x   = msg.pose.pose.position.x
            self.y   = msg.pose.pose.position.y
            self.yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

        now = self.get_clock().now().to_msg()

        # -- Publish filtered odometry -----------------------------------------
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vw

        # Fill covariance — use tight values when stopped, loose when moving
        if moving:
            odom.pose.covariance  = self.POSE_COV_MOVING
            odom.twist.covariance = self.TWIST_COV_MOVING
        else:
            odom.pose.covariance  = self.POSE_COV_STOPPED
            odom.twist.covariance = self.TWIST_COV_STOPPED

        self.odom_pub.publish(odom)

        # -- Broadcast TF: odom -> base_footprint ------------------------------
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """Extract yaw from a geometry_msgs/Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
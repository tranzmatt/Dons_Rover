#!/usr/bin/env python3
"""
odom_filter.py — Odometry noise filter for rf2o laser odometry

rf2o generates small phantom motions from scan-to-scan laser noise even when
the robot is completely stationary. This node filters those out by zeroing
velocities below a configurable threshold before broadcasting the TF that
slam_toolbox uses for pose estimation.

Subscribes:  /odom_rf2o      (nav_msgs/Odometry) — raw rf2o output
Publishes:   /odom_filtered  (nav_msgs/Odometry) — cleaned odometry
Broadcasts:  odom → base_footprint TF

Setup:
  - rf2o must have publish_tf: false (this node takes over TF broadcasting)
  - slam_toolbox uses the TF broadcast from this node

Tuning:
  - linear_threshold:  increase if map still twitches while stationary
  - angular_threshold: increase if map still rotates while stationary
  - Start conservative (small values) and increase until twitching stops
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
        # Velocities below these thresholds are treated as zero (noise floor).
        # linear_threshold:  m/s  — typical rf2o noise is ~0.001-0.003 m/s
        # angular_threshold: rad/s — typical rf2o noise is ~0.005-0.015 rad/s
        self.declare_parameter('linear_threshold',  0.005)  # m/s
        self.declare_parameter('angular_threshold', 0.02)   # rad/s
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('base_frame',        'base_footprint')

        self.linear_threshold  = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.odom_frame        = self.get_parameter('odom_frame').value
        self.base_frame        = self.get_parameter('base_frame').value

        self.get_logger().info('OdomFilter started')
        self.get_logger().info(f'  Linear threshold:  {self.linear_threshold} m/s')
        self.get_logger().info(f'  Angular threshold: {self.angular_threshold} rad/s')
        self.get_logger().info(f'  TF: {self.odom_frame} -> {self.base_frame}')

        # -- State -------------------------------------------------------------
        # Accumulated pose — we integrate only motion that clears the threshold
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
        # Extract velocities from rf2o message
        vx  = msg.twist.twist.linear.x
        vw  = msg.twist.twist.angular.z

        # Apply deadzone — treat sub-threshold motion as zero
        if abs(vx) < self.linear_threshold:
            vx = 0.0
        if abs(vw) < self.angular_threshold:
            vw = 0.0

        # If both are zero, robot is considered stationary.
        # Still publish/broadcast to keep TF tree fresh, but don't update pose.
        if vx != 0.0 or vw != 0.0:
            # Integrate velocity into pose using rf2o's own pose estimate.
            # We trust rf2o's accumulated position when motion clears threshold,
            # and freeze it when motion is below threshold.
            self.x   = msg.pose.pose.position.x
            self.y   = msg.pose.pose.position.y
            self.yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

        now = self.get_clock().now().to_msg()

        # -- Publish filtered odometry -----------------------------------------
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vw

        # Copy covariance from rf2o
        odom.pose.covariance  = msg.pose.covariance
        odom.twist.covariance = msg.twist.covariance

        self.odom_pub.publish(odom)

        # -- Broadcast TF: odom → base_footprint -------------------------------
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
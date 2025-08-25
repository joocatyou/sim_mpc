#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

class OdomTfPublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Assuming RTAB-Map publishes to /odom
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Odom TF Publisher has been started.')

    def odom_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Read message content and assign it to the transform message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # The parent frame
        t.child_frame_id = 'base_link'   # The child frame

        # Robot's position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Robot's orientation (quaternion)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_tf_publisher = OdomTfPublisher()
    rclpy.spin(odom_tf_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

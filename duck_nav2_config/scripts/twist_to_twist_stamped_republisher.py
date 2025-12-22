#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRepublisher(Node):
    """
    A node that subscribes to a Twist topic and republishes the data
    as a TwistStamped message on a different topic.
    """
    def __init__(self):
        super().__init__('twist_republisher')

        # Parameters for input and output topics
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/duck_control/cmd_vel')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Create a subscriber to the standard Nav2 cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10)

        # Create a publisher for the custom driver's topic
        self.publisher_ = self.create_publisher(
            TwistStamped,
            output_topic,
            10)

        self.get_logger().info(f"Subscribing to '{input_topic}' and republishing to '{output_topic}'")

    def twist_callback(self, msg: Twist):
        """
        Callback function that receives a Twist message and publishes it as TwistStamped.
        """
        # Create a new TwistStamped message
        twist_stamped_msg = TwistStamped()

        # Set the header with current time and frame_id
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_footprint' # Or whatever is appropriate

        # Copy the twist data
        twist_stamped_msg.twist = msg

        # Publish the new message
        self.publisher_.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_republisher = TwistRepublisher()
    rclpy.spin(twist_republisher)
    twist_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
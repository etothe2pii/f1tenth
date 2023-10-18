#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import sys
import math
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.

        self.driver_pub =  self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scanner_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.angles_array = []
        self.threshold = 1.5
        self.confidence = 50

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        if len(self.angles_array) == 0:
            angle = scan_msg.angle_min
            for i in range(len(scan_msg.ranges)):
                self.angles_array.append(math.cos(angle))
                angle += scan_msg.angle_increment
            
            self.angles_array = np.array(self.angles_array)
        
        #Calculate the rates and set negative values to 0
        rates = self.speed * self.angles_array
        rates[rates<0] = 0

        #Calculate the ittc, divisions by 0 (-inf/inf) become inf
        ranges = np.array(scan_msg.ranges)
        ittc = np.divide(ranges, rates)
        ittc[ittc==float("-inf")] = float("inf")

        # publish command to brake
        #Check if the number of measurements below the threshold is more than the confidence
        if (ittc<self.threshold).sum() > self.confidence:
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = self.get_clock().now().to_msg()
            ack_msg.drive.speed = 0.0
            self.driver_pub.publish(ack_msg)
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
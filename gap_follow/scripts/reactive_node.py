#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        #Pubs and Subs

        self.driver_pub =  self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.scanner_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)

        #Some tunable variables

        self.max_distance = 3.0
        self.average_window = 3
        self.obs_rad = 32.5
        self.car_rad = 25
        self.target_distance = 1.5
        self.speed = 0.5
        self.disparity_th = 0.25

        self.previous_angle = 0

        self.min = 0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)

        #Clip high values
        proc_ranges[proc_ranges > self.max_distance] = self.max_distance

        #Moving average
        proc_ranges = np.convolve(proc_ranges, np.ones(self.average_window), mode="valid")/self.average_window
         
        proc_ranges = np.array([proc_ranges[0], proc_ranges[0]] + list(proc_ranges) + [proc_ranges[-1], proc_ranges[-1]])

        

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # ranges_ind = np.nonzero(free_space_ranges > self.target_distance)
        # diffs = np.diff(ranges_ind)
        # ind = np.nonzero(diffs > 1 or diffs < 1)
        #free_space_ranges[free_space_ranges < self.target_distance] = 0
        max_width = 0
        start = 0
        end = 0
        in_range = False
        tmp_start = 0
        for i in range(len(free_space_ranges)):
            if not in_range:
                if free_space_ranges[i] !=0:
                    in_range = True
                    tmp_start = i
            else:
                if free_space_ranges[i] == 0:
                    in_range = False
                    
                    if i -tmp_start > max_width:
                        start = tmp_start
                        end = i-1
                        max_width = i - tmp_start
        
        if free_space_ranges[-1] != 0:
            if i - tmp_start > max_width:
                    start = tmp_start
                    end = i-1

        return start, end
            



        #return  max(len(x) for x in ranges)
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
#        farthest = np.argmax(ranges[start_i:end_i]) + start_i
        return np.argmax(ranges[start_i:end_i]) + start_i
#        dist = end_i - start_i
#        if self.min < start_i:
#            dist = end_i - farthest
#            return end_i - math.floor(dist*0.7)
#
#        dist = farthest - start_i
#        return start_i + math.floor(dist*0.7)
        #return (start_i + end_i)/2
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges



        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR

        minimum = np.argmin(proc_ranges)
        self.min = minimum
        #Eliminate all points inside 'bubble' (set them to zero) 
        rad = math.floor(self.obs_rad/max(0.1, proc_ranges[minimum]) + self.car_rad)
        proc_ranges[minimum-rad:minimum+rad] = 0

        #Find max length gap

        start, end = self.find_max_gap(proc_ranges) 

        #Find the best point in the gap

        point = self.find_best_point(start, end, proc_ranges)

        #Publish Drive message

        angle = data.angle_min
        increment = data.angle_increment

        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.drive.steering_angle = 1* (angle + point * increment)
        
        #ack_msg.drive.steering_angle = -1.0
        #ack_msg.drive.steering_angle_velocity = 1.5
        difference = abs(ack_msg.drive.steering_angle - self.previous_angle) + 1 # + 1 to prevent div by 0 errors
        #ack_msg.drive.speed = (1/difference)*0.1
        ack_msg.drive.speed = self.speed
        self.previous_angle = ack_msg.drive.steering_angle
        #print(angle, data.angle_max, increment, point)
        #print(angle/math.pi, data.angle_max/math.pi, increment, point)
        #print(angle + point * increment, (angle+point*increment)/math.pi)
        #print(angle, increment)
        #print(0, data.ranges[0])
        #print(len(data.ranges) - 1, data.ranges[-1])
        #print(math.floor(abs(angle)//increment), data.ranges[math.floor(abs(angle)//increment)])
        #print(math.floor((math.pi/2-angle)//increment), data.ranges[math.floor((math.pi/2-angle)//increment)])
        #print(math.floor((-1*math.pi/2 -angle)//increment), data.ranges[math.floor((-1*math.pi/2 -angle)//increment)])
        #print(539, angle + 539 * increment)
        #print(f"i:{point} min:{start},{angle + start * increment:.2f} max:{end},{angle + end*increment:.2f} target:{-1 * (angle + point*increment):.2f} actual:{ack_msg.drive.steering_angle}", end = "\r")
        
        print(f"{proc_ranges[point]:.2f}    ", end = "\r")
        self.driver_pub.publish(ack_msg)




def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

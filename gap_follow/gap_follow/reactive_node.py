import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

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

        self.max_distance = 3
        self.average_window = 5
        self.obs_rad = 10
        self.car_rad = 8

        self.target_distance = 2


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

        

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # ranges_ind = np.nonzero(free_space_ranges > self.target_distance)
        # diffs = np.diff(ranges_ind)
        # ind = np.nonzero(diffs > 1 or diffs < 1)
        free_space_ranges[free_space_ranges < self.target_distance] = 0
        max_width = 0
        start = 0
        end = 0
        in_range = False
        tmp_start = 0
        tmp_end = 0
        for i in range(len(free_space_ranges)):
            if not in_range:
                if free_space_ranges[i] !=0:
                    in_range = True
                    tmp_start = i
            else:
                if free_space_ranges[i] == 0:
                    in_range = False
                    
                    if i -1 -tmp_start > max_width:
                        start = tmp_start
                        end = i-1
                        max_width = i - 1 - tmp_start
        
        if free_space_ranges[-1] != 0:
            if i -1 -tmp_start > max_width:
                    start = tmp_start
                    end = i-1
            



        #return  max(len(x) for x in ranges)
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        return None

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR

        minimum = np.argmin(proc_ranges)

        #Eliminate all points inside 'bubble' (set them to zero) 

        proc_ranges[minimum-self.obs_rad, minimum+self.obs_rad] = 0

        #Find max length gap

        self.find_max_gap(proc_ranges) 

        #Find the best point in the gap 

        #Publish Drive message


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
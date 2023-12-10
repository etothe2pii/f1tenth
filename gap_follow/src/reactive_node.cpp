#include <string>
#include <vector>
#include <float.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

#define PI			3.14159265
#define BUBBLE_R	0.5
#define PUSH_SIZE	0
#define WALL_GAP_1	0.5
#define WALL_TURN_1	0.174533
#define WALL_GAP_2	0.3
#define WALL_TURN_2	0.436332

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
	ReactiveFollowGap();

private:
	// ROS subscribers and publishers
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_drivePub;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_subscription;
	std::string lidarscan_topic = "/scan";
	std::string drive_topic = "/drive";

	void preprocess_lidar(float* ranges);
	void find_max_gap(float* ranges, int* indice);
	void find_best_point(float* ranges, int* indice);
	void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

ReactiveFollowGap::ReactiveFollowGap() : Node("reactive_node") {
	// Initialize subscribers and publishers
	m_drivePub = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
	m_subscription = create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1,
			std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
}


void ReactiveFollowGap::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
	// Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
	long int m = scan_msg->ranges.size();
	double* scan = new double[m];
	int best_point = 0;

	// Find closest point to LiDAR
	long int closest_point = 0;
	double closest_dist = DBL_MAX;
	for(int i = 0; i < m; i++) {
		if(scan_msg->ranges[i] < closest_dist) {
			closest_point = i;
			closest_dist = scan_msg->ranges[i];
		}
		scan[i] = scan_msg->ranges[i];
	}
//	printf("Closest point: %ld at %f\n", closest_point, closest_dist);

	// Eliminate all points inside 'bubble' (set them to zero)
	double theta = atan(BUBBLE_R/closest_dist);
	// How many scans do we block out?
	int bubble_size = (int)(theta/scan_msg->angle_increment);

//	printf(" bubble-size=%d\n", bubble_size);

	for(int i = -bubble_size; i < bubble_size; i++) {
		long int index = closest_point+i;
		if(index >= 0 && index < m) {
			scan[index] = 0;
		}
	}

	// Find max length gap
	int gap_start, gap_end, push;
	// Which side of the bubble is the largest gap?
	if(closest_point < m/2) {
		// Largest gap is to the left
		gap_start = closest_point;
		gap_end = m;
		push = PUSH_SIZE;
	}
	else {
		// Largest gap is to the right
		gap_start = 0;
		gap_end = closest_point;
		push = -PUSH_SIZE;
	}

	// Find the best point in the gap
	double farthest_dist = 0;
	for(int i = gap_start; i < gap_end; i ++) {
		if(scan[i] > farthest_dist) {
			farthest_dist = scan[i];
			best_point = i;
		}
	}

	best_point = best_point+push;
	if(best_point < 0) {
		best_point = 0;
	}
	else if(best_point >= m) {
		best_point = m-1;
	}

	// Determine the steering angle
	double angle = (best_point - m/2)*scan_msg->angle_increment;

	double speed = 0;
	if(farthest_dist > 10) {
		speed = 5;
	}
	else if(farthest_dist > 6) {
		speed = 3.5;
	}
	else if(farthest_dist > 3.5) {
		speed = 2.5;
	}
	else if(farthest_dist > 2.5) {
		speed = 1.5;
	}
	else {
		speed = 0.5;
	}

	// Verify that we aren't about to hit the wall...
	int i_0_deg = 0.7854/scan_msg->angle_increment;
	int i_180_deg = 3.92699/scan_msg->angle_increment;

	// Check right and left sides
	if(scan_msg->ranges[i_0_deg ] < WALL_GAP_2) {
		// TURN LEFT!
		angle = 0 + WALL_TURN_1;
		speed = std::min(1.0, speed);
	}
	else if(scan_msg->ranges[i_0_deg ] < WALL_GAP_1) {
		// Turn left!
		angle += WALL_TURN_1;
		speed = std::min(2.5, speed);
	}

	if(scan_msg->ranges[i_180_deg ] < WALL_GAP_2) {
		// TURN RIGHT!
		angle = 0 - WALL_TURN_1;
		speed = std::min(1.0, speed);
	}
	else if(scan_msg->ranges[i_180_deg ] < WALL_GAP_1) {
		// Turn right!
		angle -= WALL_TURN_1;
		speed = std::min(2.5, speed);
	}

//	printf("Drive towards point %d,", best_point);
//	printf(" turn %f degrees\n", angle);

	// Publish Drive message
	ackermann_msgs::msg::AckermannDriveStamped drive_msg;
	drive_msg.drive.speed = speed;
	drive_msg.drive.steering_angle = angle;
	m_drivePub->publish(drive_msg);

}


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}

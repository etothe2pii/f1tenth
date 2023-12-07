#include <string>
#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define INTEGRAL_ARR_SIZE	5

using namespace std::chrono_literals;

class WallFollow : public rclcpp::Node {

public:
	WallFollow();
	// Callback function for LaserScan messages
	void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
	// Timer callback function
	void timer_callback();

private:
	// PID CONTROL PARAMS
	// Not bad... kp = 3.0, kd = 0.5, ki = 0.01;
	double kp = 3.0;
	double kd = 0.5;
	double ki = 0.01;
	double servo_offset = 0.0;
	double prev_error = 0.0;
	double prev_V_theta = 0.0;
	double integral = 0.0;
	const double TARGET = 1.0;
	const double AC = 1.5;
	const double speed = 1.5;
	bool turn_right = true;
	double error_array[INTEGRAL_ARR_SIZE];
	int error_i = 0;
	bool run_controller;

	// ROS things
	std::string lidarscan_topic = "/scan";
	std::string drive_topic = "/drive";
	/// ROS subscribers and publishers
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_drivePub;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_subscription;
	rclcpp::TimerBase::SharedPtr timer_;

	// Returns the corresponding range measurement at a given angle
	double get_range(float* range_data, double angle);
	// Calculates the error to the wall
	double get_error(float* range_data, double dist);
	// Publishes vehicle control based on the calculated error
	void pid_control(double error, double velocity);
};

// Constructor
WallFollow::WallFollow() : Node("wall_follow_node") {
	// Initialize subscribers and publishers
	m_drivePub = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
	m_subscription = create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1,
			std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
	timer_ = create_wall_timer(150ms, std::bind(&WallFollow::timer_callback, this));
	// Initialize error array
	for(int i = 0; i < INTEGRAL_ARR_SIZE; i++) {
		error_array[i] = 0;
	}
	run_controller = true;
}

// Timer call-back - update the drive command
void WallFollow::timer_callback() {
	run_controller = true;
}

// Callback function for LaserScan messages
void WallFollow::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
	if(run_controller) {
		/*
		Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

		Args:
			msg: Incoming LaserScan message

		Returns:
			None

		* LiDAR goes from +135-deg to -135-deg, where 0 is straight infront of car. We want "0-deg"
		* to be the right angle from the car, this means we need to grab the measurement 45-deg
		* (0.7854 rad) from the first measurement.
		*/
		double new_V_theta = 0.0;
		double error = 0.0;

		if(turn_right) {
			// Use "right side" of car as 0-deg
			int i_0_deg = 0.7854/scan_msg->angle_increment;
			double b = scan_msg->ranges[i_0_deg];

			/*
			 * alpha = tan inverse ( a cos(theta) - b/ a sin(theta) )
			 * AB = b cos(alpha)
			 *
			 * We pick theta = 35-deg (0.6109)
			 *
			 * Actual error
			 * AB + AC*sin(alpha)
			 */
			double theta = 0.6109;
			int i_35_deg = theta/scan_msg->angle_increment + i_0_deg;
			double a = scan_msg->ranges[i_35_deg];
			double alpha = atan((a*cos(theta) - b)/( a*sin(theta)));

			double AB = b*cos(alpha);
			double CD = AB + AC*sin(alpha);

			// Add up cumulative error over time
			double cumu_error = 0;
			for(int i = 0; i < INTEGRAL_ARR_SIZE; i++) {
				cumu_error += error_array[i];
			}

			// Run PID
			error = TARGET-CD;
			double V_theta = kp*error + kd*(prev_error-error) + ki*cumu_error;
			// New output
			new_V_theta = V_theta-prev_V_theta;
		}
		else {
			// Turn left! Use left side of car as 0-deg
			int i_0_deg = 3.926990817/scan_msg->angle_increment;
			double b = scan_msg->ranges[i_0_deg];

			/*
			 * alpha = tan inverse ( a cos(theta) - b/ a sin(theta) )
			 * AB = b cos(alpha)
			 *
			 * We pick theta = 35-deg (0.6109)
			 *
			 * Actual error
			 * AB + AC*sin(alpha)
			 */
			double theta = 0.610865238;
			int i_35_deg = 3.316125579/scan_msg->angle_increment;
			double a = scan_msg->ranges[i_35_deg];
			double alpha = atan((a*cos(theta) - b)/( a*sin(theta)));

			double AB = b*cos(alpha);
			double CD = AB + AC*sin(alpha);

			// Run PID
			error = TARGET-CD;
			double V_theta = kp*error + kd*(prev_error-error);
			// New output
			new_V_theta = prev_V_theta-V_theta;
		}

		// Send Ackermann message with new output
		ackermann_msgs::msg::AckermannDriveStamped drive_msg;
		drive_msg.drive.speed = speed;
		drive_msg.drive.steering_angle = new_V_theta;
		m_drivePub->publish(drive_msg);

		// Update previous with new inputs
		prev_V_theta = new_V_theta;
		prev_error = error;
		error_array[error_i%INTEGRAL_ARR_SIZE] = prev_error;
		error_i++;

		// Reset timer flag
		run_controller = false;
	}
}

// Returns the corresponding range measurement at a given angle
double WallFollow::get_range(float* range_data, double angle) {
	/*
	Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

	Args:
		range_data: single range array from the LiDAR
		angle: between angle_min and angle_max of the LiDAR

	Returns:
		range: range measurement in meters at the given angle
	*/



	return 0.0;
}

// Calculates the error to the wall
double WallFollow::get_error(float* range_data, double dist) {
	/*
	Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

	Args:
		range_data: single range array from the LiDAR
		dist: desired distance to the wall

	Returns:
		error: calculated error
	*/

	// TODO:implement
	return 0.0;
}

// Publishes vehicle control based on the calculated error
void WallFollow::pid_control(double error, double velocity) {
	/*
	Based on the calculated error, publish vehicle control

	Args:
		error: calculated error
		velocity: desired velocity

	Returns:
		None
	*/
	double angle = 0.0;
	// TODO: Use kp, ki & kd to implement a PID controller
	auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
	// TODO: fill in drive message and publish
}


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollow>());
	rclcpp::shutdown();
	return 0;
}

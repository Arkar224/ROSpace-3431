// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>


using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

	robot_pose_ = 0.0;
	near_start = false;

	last_left_front_ = 0.0;
	marker_detected_ = false;
	marker_log_.open("/home/ubuntu/turtlebot3_ws/src/wall_follower/landmarks.csv", std::ios::out | std::ios::trunc);
	marker_log_ << "x,y\n";

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	// keep last 10 msgs as buffer
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	// - what topic we are publishing to
	// - twist is a type of msg for motor speed
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	// do the actual work 
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Helper function
********************************************************************************/
bool WallFollower::detect_cylinder_marker()
{
    double left_front = scan_data_[LEFT_FRONT];
    
    // If previous scan was stable, detect a sudden close object (marker)
    if (!marker_detected_ && left_front < 0.25 && last_left_front_ > 0.4) {
        marker_detected_ = true;  // avoid multiple logs for same cylinder
        return true;
    }

    // Once we’ve passed it (distance returns to normal), reset flag
    if (marker_detected_ && left_front > 0.4) {
        marker_detected_ = false;
    }

    last_left_front_ = left_front;
    return false;
}

void WallFollower::log_marker_position()
{
    if (marker_log_.is_open()) {
        marker_log_ << current_x_ << "," << current_y_ << "\n";
        marker_log_.flush();
    }
    RCLCPP_INFO(this->get_logger(), "Cylinder marker detected at (%.2f, %.2f)", current_x_, current_y_);
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.32

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;

	// fprintf(stderr, "Start (x,y) = (%.2f, %.2f), Current (x,y) = (%.2f, %.2f)\n", start_x, start_y, current_x, current_y);
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 15

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	// deal with the sector left and right to zero bit 
	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (0 < msg->ranges.at(angle) && msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (0 < msg->ranges.at(angle) && msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	// other sectors
	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (0 < msg->ranges.at(angle) && msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;


void WallFollower::update_callback()
{
    // FILE *fp = fopen("scan_data_log.txt", "a");
    if (near_start) {
        update_cmd_vel(0.0, 0.0);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: Near start, stopping\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        //     fclose(fp);
        // }
        exit(0);
    } else if (scan_data_[LEFT_FRONT] > 0.7) {
        // fprintf(stderr, "Lost wall on left front\n");
        update_cmd_vel(0.08, 1.0);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: Lost wall on left front\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        // }
    } else if (scan_data_[FRONT] < 0.45) {
        // fprintf(stderr, "Blocked front\n");
        update_cmd_vel(0.0, -0.6);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: Blocked front\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        // }
    } else if (scan_data_[FRONT_LEFT] < 0.38) {
        // fprintf(stderr, "Blocked front left\n");
        update_cmd_vel(0.12, -0.5);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: Blocked front left\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        // }
    } else if (scan_data_[FRONT_RIGHT] < 0.46) {
        // fprintf(stderr, "Blocked front right\n");
        update_cmd_vel(0.12, 0.5);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: Blocked front right\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        // }
    } else if (scan_data_[LEFT_FRONT] > 0.42) {
        // fprintf(stderr, "LF within range\n");
        update_cmd_vel(0.12, 0.3);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: LF within range\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        // }
    } else {
        // fprintf(stderr, "No obstacles detected\n");
        update_cmd_vel(0.2, 0.0);
        // if (fp) {
        //     fprintf(fp, "FR: %.2f, F: %.2f, LF: %.2f, FL: %.2f | Decision: No obstacles detected\n",
        //         scan_data_[FRONT_RIGHT], scan_data_[FRONT], scan_data_[LEFT_FRONT], scan_data_[FRONT_LEFT]);
        // }
    }
    // if (fp) fclose(fp);

	if (detect_cylinder_marker()) {
        log_marker_position();
    }
}



/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	// spin will just keep looping
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}

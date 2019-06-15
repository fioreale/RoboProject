#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "nav_msgs/Odometry.h"
#include "imu_complementary_filter/complementary_filter_ros.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define DISTANCE 1.765
#define STEER_FACT 18

class Odometry {

	private:
		ros::NodeHandle nh;
		ros::Publisher pub;

		// computed odometry
		double odom_x;
		double odom_y;
		double odom_theta;

		// last timestamp measured
		ros::Time last_time;

	public:
  	Odometry() {
		set_last_time(ros::Time::now());
		set_x(0);
		set_y(0);
		set_theta(0);
		set_type(0);
	}

	void set_x(double x){
		this->odom_x = x;
	}

	void set_y(double y){
		this->odom_y = y;
	}

	void set_theta(double theta){
		this->odom_theta = theta;
	}

	void set_last_time(ros::Time time){
		this->last_time = time;
	}

	void set_type(double type){
		this->odom_type = type;
	}

	double get_x(){
		return odom_x;
	}

	double get_y(){
		return odom_y;
	}

	double get_theta(){
		return odom_theta;
	}

	ros::Time get_last_time(){
		return last_time;
	}

	int get_type(){
		return odom_type;
	}

	//computation of the ackerman steering odometry
	void ackerman(double v, double steering_angle, double delta_t) {

		steering_angle *= M_PI / (180 * STEER_FACT);

		double old_x = get_x();
		double old_y = get_y();
		double old_theta = get_theta();

		double r = DISTANCE / tan(steering_angle);
		double omega = (v * sin(steering_angle)) / DISTANCE;
		double d_theta = omega * delta_t;

		double _d_x = r * sin(d_theta);
		double _d_y = r * (1 - cos(d_theta));
		double d_x = _d_x*cos(old_theta) - _d_y*sin(old_theta);
		double d_y = _d_x*sin(old_theta) + _d_y*cos(old_theta);

		double new_x = old_x + d_x;
		double new_y = old_y + d_y;
		double new_theta = fmod(old_theta + d_theta, 2*M_PI);

		// save the updated odometry
		set_x(new_x);
		set_y(new_y);
		set_theta(new_theta);
	}

	// odometry callback
	void callback(const project_1::floatStamped::ConstPtr& steer, const project_1::floatStamped::ConstPtr& speed_R) {
		// retrive odometry type parameter
		// do the math ...
		ros::Time current_time = ros::Time::now();
		ros::Duration diff_time = current_time - this->get_last_time();

		ackerman(speed_L->data, speed_R->data, steer->data, diff_time.toSec());

		// set and publish odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		// set the position
		odom.pose.pose.position.x = get_x();
		odom.pose.pose.position.y = get_y();
		odom.pose.pose.position.z = 0.0;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(get_theta());

		odom.pose.pose.orientation = odom_quat;

		// publish the odometry
		pub = nh.advertise<nav_msgs::Odometry>("/odom_topic", 100);
		pub.publish(odom);

		//ROS_INFO("Received the message [v(l) - v(r) - steer]: [%f - %f - %f]", speed_L->data,speed_R->data,steer->data);

		ROS_INFO("Computed pose (type: %d) [x - y - theta]: [%f - %f - %f]",get_type(),get_x(),get_y(),get_theta()*180/M_PI);

		//set the last useful time
		set_last_time(current_time);
	}

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "project_2");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  imu_tools::ComplementaryFilterROS filter(nh, nh_private);

  Odometry odom;

  ros::NodeHandle nh;

	// subscriber for odometry
	message_filters::Subscriber<project_1::floatStamped> sub1(nh, "/steer_stamped", 100);
	message_filters::Subscriber<project_1::floatStamped> sub2(nh, "/speedL_stamped", 100);

	typedef message_filters::sync_policies::ApproximateTime<project_1::floatStamped, project_1::floatStamped, project_1::floatStamped> MySyncPolicy;

	// creation of Synchronizer and relative callback binding
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
	sync.registerCallback(boost::bind(&Odometry::callback, &odom, _1, _2));

  ros::spin();
  return 0;
}

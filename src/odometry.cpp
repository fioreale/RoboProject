#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "dynamic_reconfigure/server.h"
#include "project_1/floatStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/Odometry.h"
#include "project_1/parametersConfig.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define BASE_L 1.765
#define DISTANCE 1.3
#define STEER_FACT 18

class Odometry {

	private:
		ros::NodeHandle nh;
		tf::TransformBroadcaster br;
		ros::Subscriber sub;
		ros::Publisher pub;

		int odom_type;

		double odom_x;
		double odom_y;
		double odom_theta;

		ros::Time last_time;

	public:
  	Odometry() {

		set_last_time(ros::Time::now());
		set_x(0);
		set_y(0);
		set_theta(0);
		set_type(0);
	/*	message_filters::Subscriber<project_1::floatStamped> sub1(nh, "/speedL_stamped", 100);
		message_filters::Subscriber<project_1::floatStamped> sub2(nh, "/speedR_stamped", 100);
		message_filters::Subscriber<project_1::floatStamped> sub3(nh, "/steer_stamped", 100);

		typedef message_filters::sync_policies::ApproximateTime<project_1::floatStamped, project_1::floatStamped, project_1::floatStamped> MySyncPolicy;

		//creation of Synchronizer and relative callback binding
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
		sync.registerCallback(boost::bind(&Odometry::callback, _1, _2, _3));

		dynamic_reconfigure::Server<project_1::parametersConfig> server;
		dynamic_reconfigure::Server<project_1::parametersConfig>::CallbackType f;
		f = boost::bind(&Odometry::param_callback, _1, _2);
		server.setCallback(f);*/
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

	void diff_drive(double vl, double vr, double delta_t) {
		//computation of the Differential Drive odometry
		double old_x = get_x();
		double old_y = get_y();
		double old_theta = get_theta();

		double avg_v = (vl + vr) / 2;
		double omega = (vr - vl) / BASE_L;

		double new_x;
		double new_y;
		double new_theta = fmod(old_theta + omega*delta_t, 2*M_PI);

		if(omega != 0) {
			new_x = odom_x + (avg_v/omega)*(cos(new_theta) - cos(odom_theta));
			new_y = odom_y + (avg_v/omega)*(sin(new_theta) - sin(odom_theta));
		}
		else {
			new_x = odom_x + avg_v * delta_t * cos(new_theta);
			new_y = odom_y + avg_v * delta_t * sin(new_theta);
		}

		set_x(new_x);
		set_y(new_y);
		set_theta(new_theta);

		tf::Transform tf_car, tf_wheel_fl, tf_wheel_fr;

		tf_car.setOrigin(tf::Vector3(new_x, new_y, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, new_theta);
		tf_car.setRotation(q);

		tf_wheel_fl.setOrigin(tf::Vector3(new_x - BASE_L/2*cos(new_theta), new_y - BASE_L/2*sin(new_theta), 0));
		tf_wheel_fl.setRotation(q);

		tf_wheel_fr.setOrigin(tf::Vector3(new_x + BASE_L/2*cos(new_theta), new_y + BASE_L/2*sin(new_theta), 0));
		tf_wheel_fr.setRotation(q);

		ros::Time tf_time = ros::Time::now();

		this->br.sendTransform(tf::StampedTransform(tf_car, tf_time, "world", "car"));
		this->br.sendTransform(tf::StampedTransform(tf_wheel_fl, tf_time, "car", "wheel_FL"));
		this->br.sendTransform(tf::StampedTransform(tf_wheel_fr, tf_time, "car", "wheel_FR"));
	}

	void ackerman(double vl, double vr, double steering_angle, double delta_t) {
		steering_angle *= M_PI / (180 * STEER_FACT);
		double steering_angle_l;
		double steering_angle_r;

		double old_x = get_x();
		double old_y = get_y();
		double old_theta = get_theta();

		double r = DISTANCE / tan(steering_angle);
		double avg_v = (vl + vr) / 2;
		double omega = (avg_v * sin(steering_angle)) / DISTANCE;
		double d_theta = omega * delta_t;
		double d_x = r * (1 - cos(d_theta)) * cos(old_theta);
		double d_y = r * sin(d_theta) * sin(old_theta);

		double new_x = old_x + d_x;
		double new_y = old_y + d_y;
		double new_theta = fmod(old_theta + d_theta, 2*M_PI);

		double x_fr, y_fr, x_br, y_br;
		double x_fl, y_fl, x_bl, y_bl;

		double axis_x, axis_y;

		set_x(new_x);
		set_y(new_y);
		set_theta(new_theta);

		tf::Transform tf_car, tf_wheel_fl, tf_wheel_fr, tf_wheel_bl, tf_wheel_br;
		//in the forward part the 'transform' string has to be adjusted for every tf component(car, wheels...)
		tf_car.setOrigin(tf::Vector3(new_x, new_y, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, new_theta);
		//q= q.normalized();
		tf_car.setRotation(q);
		tf_wheel_fl.setRotation(q);
		tf_wheel_fr.setRotation(q);

		x_bl = new_x - BASE_L/2*cos(new_theta);
		y_bl = new_y - BASE_L/2*sin(new_theta);

		tf_wheel_bl.setOrigin(tf::Vector3(x_bl, y_bl, 0));
		tf_wheel_bl.setRotation(q);

		x_br = new_x + BASE_L/2*cos(new_theta);
		y_br = new_y + BASE_L/2*sin(new_theta);

		tf_wheel_br.setOrigin(tf::Vector3(x_br, y_br, 0));
		tf_wheel_br.setRotation(q);

		axis_x = DISTANCE*cos(new_theta);
		axis_y = DISTANCE*sin(new_theta);

		x_fr = x_br + axis_x;
		y_fr = y_br + axis_y;

		x_fl = x_bl + axis_x;
		y_fl = y_bl + axis_y;

		tf::Quaternion q_fr;
		tf::Quaternion q_fl;

		if(steering_angle > 0){
			steering_angle_l = atan(DISTANCE / (r + BASE_L));
			steering_angle_r = atan(DISTANCE / (r - BASE_L));
		}
		else {
			steering_angle_l = atan(DISTANCE / (r - BASE_L));
			steering_angle_r = atan(DISTANCE / (r + BASE_L));
		}

		q_fr.setRPY(0, 0, new_theta - steering_angle_r);
		q_fl.setRPY(0, 0, new_theta - steering_angle_l);

		tf_wheel_fr.setOrigin(tf::Vector3(y_fr, y_fr, 0));
		tf_wheel_fr.setRotation(q);

		tf_wheel_fl.setOrigin(tf::Vector3(x_fl, x_fl, 0));
		tf_wheel_fl.setRotation(q);

		ros::Time tf_time = ros::Time::now();

		this->br.sendTransform(tf::StampedTransform(tf_car, tf_time, "world", "car"));
		this->br.sendTransform(tf::StampedTransform(tf_wheel_fl, tf_time, "car", "wheel_FL"));
		this->br.sendTransform(tf::StampedTransform(tf_wheel_fr, tf_time, "car", "wheel_FR"));
		this->br.sendTransform(tf::StampedTransform(tf_wheel_bl, tf_time, "car", "wheel_BL"));
		this->br.sendTransform(tf::StampedTransform(tf_wheel_br, tf_time, "car", "wheel_BR"));
	}

	// odometry callback
	void callback(const project_1::floatStamped::ConstPtr& speed_L, const project_1::floatStamped::ConstPtr& speed_R, const project_1::floatStamped::ConstPtr& steer) {
		// retrive odometry type parameter
		// do the math ...

		ros::Time current_time = ros::Time::now();
		ros::Duration diff_time = current_time - this->get_last_time();

		switch (get_type()) {
			case 0: {
				//Diff_drive
				diff_drive(speed_L->data, speed_R->data, diff_time.toSec());
				break;
			}
			case 1: {
				//Ackerman
				ackerman(speed_L->data, speed_R->data, steer->data, diff_time.toSec());
				break;
			}
		}

		// set and publish odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		// set the position
		odom.pose.pose.position.x = get_x();
		odom.pose.pose.position.y = get_y();
		odom.pose.pose.position.z = 0.0;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(get_theta());

		odom.pose.pose.orientation= odom_quat;
		pub = nh.advertise<nav_msgs::Odometry>("/odom_topic", 100);
		pub.publish(odom);

	//	ROS_INFO("Received the message [v(l) - v(r) - steer]: [%f - %f - %f]", speed_L->data,speed_R->data,steer->data);

		ROS_INFO("Computed pose (type: %d) [x - y - theta]: [%f - %f - %f]",get_type(),get_x(),get_y(),get_theta()*180/M_PI);

		//set the last useful time
		set_last_time(current_time);
	}

	//dynamic configuration
	void param_callback(project_1::parametersConfig &config, uint32_t level) {
		//change either odom type or xy-position odometry
		if(level == 0) {
			// change odom type
			set_type(config.odom_type);
		}
		else if(level == 1) {
			// change xy-position
			set_x(config.position_x);
			set_y(config.position_y);
		}
		ROS_INFO("Reconfigure Request: %d %f %f %d", config.odom_type, config.position_x, config.position_y, level);
	}

};


int main(int argc, char **argv) {
	ros::init(argc, argv, "project_1");
	Odometry odom;

  ros::NodeHandle nh;

  message_filters::Subscriber<project_1::floatStamped> sub1(nh, "/speedL_stamped", 100);
  message_filters::Subscriber<project_1::floatStamped> sub2(nh, "/speedR_stamped", 100);
  message_filters::Subscriber<project_1::floatStamped> sub3(nh, "/steer_stamped", 100);

  typedef message_filters::sync_policies::ApproximateTime<project_1::floatStamped, project_1::floatStamped, project_1::floatStamped> MySyncPolicy;

  //creation of Synchronizer and relative callback binding
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
  sync.registerCallback(boost::bind(&Odometry::callback,&odom, _1, _2, _3));
  dynamic_reconfigure::Server<project_1::parametersConfig> server;
  dynamic_reconfigure::Server<project_1::parametersConfig>::CallbackType f;
  f = boost::bind(&Odometry::param_callback,&odom, _1, _2);
  server.setCallback(f);

	ros::spin();
	return 0;
}

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "custom_messages/floatStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "project_1/parametersConfig.h"

#include <math.h>

class Odometry {

	private:
		ros::NodeHandle nh;
		tf::TransformBroadcaster br;
		ros::Subscriber sub;
		ros::Publisher pub;

		int odom_type;
		double base_L = 1.3;
		double d = 1.765;
		int steer_fact = 18;

		double odom_x;
		double odom_y;
		double odom_theta;

		ros::Time last_time;
		// current odometry
		// odom type

	public:
  	Odometry() {
		message_filters::Subscriber<custom_messages::floatStamped> sub1(nh, "/speedL_stamped", 100);
		message_filters::Subscriber<custom_messages::floatStamped> sub2(nh, "/speedR_stamped", 100);
		message_filters::Subscriber<custom_messages::floatStamped> sub3(nh, "/steer_stamped", 100);

		typedef message_filters::sync_policies::ApproximateTime<custom_messages::floatStamped, custom_messages::floatStamped, custom_messages::floatStamped> MySyncPolicy;

		//creation of Synchronizer and relative callback binding
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
		sync.registerCallback(boost::bind(&Odometry::callback, _1, _2, _3));

		dynamic_reconfigure::Server<parameter_test::parametersConfig> server;
		dynamic_reconfigure::Server<parameter_test::parametersConfig>::CallbackType f;
		f = boost::bind(&Odometry::param_callback, _1, _2);
		server.setCallback(f);
	}

	void set_x(double x){
		this.odom_x = x;
	}

	void set_y(double y){
		this.odom_y = y;
	}

	void set_theta(double theta){
		this.odom_theta = theta;
	}

	void set_last_time(ros::Time time){
		this.last_time = time;
	}

	void set_type(double type){
		this.odom_type = type;
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

		double avg_v = (vl + vr)/2;
		double omega = (vr - vl)/base_L;

		double new_x;
		double new_y;
		double theta_rad = odom_theta + omega*delta_t;

		if(omega != 0) {
			new_x = odom_x + (avg_v/omega)*(cos(theta_rad) - cos(odom_theta));
			new_y = odom_y + (avg_v/omega)*(sin(theta_rad) - sin(odom_theta));
		}
		else {
			new_x = odom_x + avg_v * delta_t * cos(theta_rad);
			new_y = odom_y + avg_v * delta_t * sin(theta_rad);
		}

		set_x(new_x);
		set_y(new_y);
		set_theta(theta_rad);

		tf::Transform tf_car, tf_wheel_fl, tf_wheel_fr;

		tf_car.setOrigin(tf::Vector3(new_x, new_y, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, new_theta);
		tf_car.setRotation(q);

		tf_wheel_fl.setOrigin(tf::Vector3(new_x - 1.3/2*cos(new_theta), new_y - 1.3/2*sin(new_theta), 0));
		tf_wheel_fl.setRotation(q);

		tf_wheel_fr.setOrigin(tf::Vector3(new_x + 1.3/2*cos(new_theta), new_y + 1.3/2*sin(new_theta), 0));
		tf_wheel_fr.setRotation(q);

		this.br.sendTransform(tf::StampedTransform(tf_car, ros::Time::now(), "world", "car"));
		this.br.sendTransform(tf::StampedTransform(tf_wheel_fl, ros::Time::now(), "car", "wheel_FL"));
		this.br.sendTransform(tf::StampedTransform(tf_wheel_fr, ros::Time::now(), "car", "wheel_FR"));
	}

	void ackerman(double vl, double vr, double steering_angle, double delta_t) {
		steering_angle *= PI / (180 * 18);

		double old_x = get_x();
		double old_y = get_y();
		double old_theta = get_theta();

		double r = 1.765 / tan(steering_angle);
		double avg_v = (vl + vr) / 2;
		double omega = (avg_v * sin(steering_angle)) / 1.765;
		double d_theta = omega * delta_t;
		double d_x = r * (1 - cos(d_theta)) * cos(old_theta);
		double d_y = r * sin(d_theta) * sin(old_theta);

		double new_x = old_x + d_x;
		double new_y = old_y + d_y;
		double new_theta = old_theta + d_theta;

		set_x(new_x);
		set_y(new_y);
		set_theta(theta_rad);

		tf::Transform tf_car, tf_wheel_fl, tf_wheel_fr, tf_wheel_bl, tf_wheel_br;
		//in the forward part the 'transform' string has to be adjusted for every tf component(car, wheels...)
		transform.setOrigin(tf::Vector3(new_x, new_y, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, new_theta);
		transform.setRotation(q);

		this.br.sendTransform(tf::StampedTransform(tf_car, ros::Time::now(), "world", "car"));
		this.br.sendTransform(tf::StampedTransform(tf_wheel_fl, ros::Time::now(), "car", "wheel_FL"));
		this.br.sendTransform(tf::StampedTransform(tf_wheel_fr, ros::Time::now(), "car", "wheel_FR"));
		// ackermann only
		this.br.sendTransform(tf::StampedTransform(tf_wheel_bl, ros::Time::now(), "car", "wheel_BL"));
		this.br.sendTransform(tf::StampedTransform(tf_wheel_br, ros::Time::now(), "car", "wheel_BR"));
	}

	// odometry callback
	void callback(const custom_messages::floatStamped::ConstPtr& speed_L, const custom_messages::floatStamped::ConstPtr& speed_R, const custom_messages::floatStamped::ConstPtr& steer) {
		// retrive odometry type parameter
		// do the math ...

		ros::Time current_time = ros::Time::now();
		ros::Time diff_time = current - this.get_last_time();

		switch (get_type()) {
			case 0: {
				//Diff_drive
				diff_drive(speed_L->data, speed_R->data, diff_time);
				//ROS_INFO("updating the position [x - y - theta]: [%f - %f - %f]", odom_x,odom_y,odom_theta);
				break;
			}
			case 1: {
				//Ackerman
				ackerman(speed_L->data, speed_R->data, steer->data, diff_time);
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
		tf::Quaternion odom_quat;
		odom_quat.setRPY(0, 0, get_theta());
		odom.pose.pose.orientation = odom_quat;

		pub = nh.advertise<nav_msgs::Odometry>("/odometry", 100);
		pub.publish(odom);

		//ROS_INFO("Received the message [v(l) - v(r) - steer]: [%f - %f - %f]", speed_L->data,speed_R->data,steer->data)

		//set the last useful time
		set_last_time(current_time);
	}

	//dynamic configuration
	void param_callback(parameter_test::parametersConfig &config, uint32_t level) {
		//change either odom type or xy-position odometry
		if(level == 0) {
			// change odom type
			set_type(config.odom_type);
		}
		else (level == 1) {
			// change xy-position
			set_x(config.position_x);
			set_y(config.position_y);
		}
		//ROS_INFO("Reconfigure Request: %d %f %f", config.int_param, config.float_param, config.float_param);
	}

};


int main(int argc, char **argv) {
	ros::init(argc, argv, "project_1");
	Odometry odom;

	ros::spin();
	return 0;
}

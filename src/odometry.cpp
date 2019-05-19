#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "custom_messages/floatStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "project_1/parametersConfig.h"

class Odometry {

	private:
		ros::NodeHandle nh;
		tf::TransformBroadcaster br;
		ros::Subscriber sub;
		ros::Publisher pub;

		int odom_type;
		float base_L = 1.3;
		float d = 1.765;

		float odom_x;
		float odom_y;
		float odom_theta;

		ros::Time last_time;
		// current odometry
		// odom type

	public:
  	Odometry() {
		message_filters::Subscriber<custom_messages::floatStamped> sub1(nh, "/speedL_stamped", 100);
		message_filters::Subscriber<custom_messages::floatStamped> sub2(nh, "/speedR_stamped", 100);
		message_filters::Subscriber<custom_messages::floatStamped> sub3(nh, "/steer_stamped", 100);

		typedef message_filters::sync_policies::ApproximateTime<custom_messages::floatStamped, custom_messages::floatStamped, custom_messages::floatStamped> MySyncPolicy;

		//creation of Synchronizer and realitive callback binding
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
		sync.registerCallback(boost::bind(&Odometry::callback, _1, _2, _3));

		dynamic_reconfigure::Server<parameter_test::parametersConfig> server;
		dynamic_reconfigure::Server<parameter_test::parametersConfig>::CallbackType f;
		f = boost::bind(&Odometry::param_callback, _1, _2);
		server.setCallback(f);
	}

	void set_x(float x){
		Odometry->odom_x = x;
	}

	void set_y(float y){
		Odometry->odom_y = y;
	}

	void set_alfa(float theta){
		Odometry->odom_theta = theta;
	}

	void set_last_time(ros::Time time){
		Odometry->last_time = time;
	}

	void set_type(float type){
		Odometry->odom_type = type;
	}

	float get_x(){
		return odom_x;
	}

	float get_y(){
		return odom_y;
	}

	float get_theta(){
		return odom_theta;
	}

	ros::Time get_last_time(){
		return last_time;
	}

	int get_type(){
		return odom_type;
	}

	void diff_drive(float vl, float vr, float delta_t){
		//computation of the Differential Drive odometry
		float avg_v = (vl + vr)/2;
		float omega = (vr - vl)/base_L;

		float new_x;
		float new_y;
		float theta_rad = odom_theta + omega*delta_t;;

		if(omega != 0){
			new_x = odom_x + (avg_v/omega)*(cos(theta_rad) - cos(odom_theta));
			new_y = odom_y + (avg_v/omega)*(sin(theta_rad) - sin(odom_theta));
		}
		else{
			new_x = odom_x + avg_v * delta_t * cos(theta_rad);
			new_y = odom_y + avg_v * delta_t * sin(theta_rad);
		}

		set_x(new_x);
		set_y(new_y);
		set_alfa(theta_rad);
	}

	void callback(const custom_messages::floatStamped::ConstPtr& speed_L, const custom_messages::floatStamped::ConstPtr& speed_R, const custom_messages::floatStamped::ConstPtr& steer) {
		// retrive odometry type parameter
		// do the math ...
		ros::Time current = ros::Time::now();
		ros::Time diff = current - Odometry->get_last_time();

		tf::Transform tf_car, tf_wheel_fl, tf_wheel_fr, tf_wheel_bl, tf_wheel_br;
		//in the forward part the 'transform' string has to be adjusted for every tf component(car, wheels...)
		transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, msg->theta);
		transform.setRotation(q);

		br.sendTransform(tf::StampedTransform(tf_car, ros::Time::now(), "world", "car"));
		br.sendTransform(tf::StampedTransform(tf_wheel_fl, ros::Time::now(), "car", "wheel_FL"));
		br.sendTransform(tf::StampedTransform(tf_wheel_fr, ros::Time::now(), "car", "wheel_FR"));
		// ackermann only
		br.sendTransform(tf::StampedTransform(tf_wheel_bl, ros::Time::now(), "car", "wheel_BL"));
		br.sendTransform(tf::StampedTransform(tf_wheel_br, ros::Time::now(), "car", "wheel_BR"));

		// publish odometry
		pub = nh.advertise<custom_messages::floatStamped>("/odometry", 100);
		pub.publish(msg);

		ROS_INFO("Received the message [v(l) - v(r) - steer]: [%f - %f - %f]", speed_L->data,speed_R->data,steer->data)

		switch (get_type()) {
			case 0: {
				//Diff_drive
				diff_drive(speed_L->data,speed_R->data,diff.toSec());
				ROS_INFO("updating the position [x - y - theta]: [%f - %f - %f]", odom_x,odom_y,odom_theta);
				break;
			}
			case 1:{
				//Ackerman
				break;
			}
		}

		//setting the last useful time
		set_last_time(current);
	}

	//dynamic configuration
	void param_callback(parameter_test::parametersConfig &config, uint32_t level) {
		//change either odom type or xy-position odometry
		ROS_INFO("Reconfigure Request: %d %f %f", config.int_param, config.float_param, config.float_param);
	}

};


int main(int argc, char **argv) {
	ros::init(argc, argv, "project_1");
	Odometry odom;

	ros::spin();
	return 0;
}

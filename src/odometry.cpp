#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "custom_messages/floatStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

class Odometry {

	public:
  	Odometry() {
		message_filters::Subscriber<custom_messages::floatStamped> sub1(nh, "/speedL_stamped", 100);
		message_filters::Subscriber<custom_messages::floatStamped> sub2(nh, "/speedR_stamped", 100);
		message_filters::Subscriber<custom_messages::floatStamped> sub3(nh, "/steer_stamped", 100);

		typedef message_filters::sync_policies::ApproximateTime<custom_messages::floatStamped, custom_messages::floatStamped, custom_messages::floatStamped> MySyncPolicy;

		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
		sync.registerCallback(boost::bind(&Odometry::callback, _1, _2, _3));

		dynamic_reconfigure::Server<parameter_test::parametersConfig> server;
		dynamic_reconfigure::Server<parameter_test::parametersConfig>::CallbackType f;
		f = boost::bind(&Odometry::param_callback, _1, _2);
		server.setCallback(f);
	}

	void callback(const custom_messages::floatStamped::ConstPtr& speed_L, const custom_messages::floatStamped::ConstPtr& speed_R, const custom_messages::floatStamped::ConstPtr& steer) {
		// retrive odometry type parameter
		// do the math ...
		tf::Transform tf_car, tf_wheel_fl, tf_wheel_fr, tf_wheel_bl, tf_wheel_br;
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
	}

	void param_callback(parameter_test::parametersConfig &config, uint32_t level) {
		//change either odom type or xy-position odometry
	}

	private:
	ros::NodeHandle nh;
	tf::TransformBroadcaster br;
	ros::Subscriber sub;
	ros::Publisher pub;
	// current odometry
	// odom type
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "project_1");
	Odometry odom;

	ros::spin();
	return 0;
}

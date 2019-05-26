
HOMEWORK PROJECT OF ROBOTICS - A.A. 2018/2019

Members :

Barbera Chiara 			-- person code : 10691187 - ID : 920938
Fiorentino Alessio 	-- person code : 10495770 - ID : 920488
Francavilla	Simone 	-- person code : 10532021 - ID : 920142



-------------------------------------------------------------------------------------------


Inside the archive:

--> project_1: the package of the project, that contains:

	 -the node code odometry.cpp in the src folder, the CMakeLists.txt and the package.xml files.
	 -the launch folder with the odometry.launch file
	 -the msg folder with the floatStamped.msg file and the odom_cm.msg file (used to publish the odometry)
	 -the cfg folder, with the parameters.cfg file (used in the dynamic reconfiguration)

--> this README.txt file

--> two images of the TF trees generated for the differential drive and the Ackerman
    odometries

-------------------------------------------------------------------------------------------

Parameters and custom messages:

To change the odometry, set the parameter odom_type :
	- odom_type = 0 --> differential drive
	- odom_type = 1 --> Ackermann

To change the origin , set the parameters position_x , position_y (Float64 variables)


Custom message used to publish the odometry :
         string		     odom_type
	 nav_msgs/Odometry   odom

-------------------------------------------------------------------------------------------

Instructions:

How to execute the code :
	- run one of the bags
	- in a separate terminal, run the node with "rosrun project_1 odometry" (or "roslaunch project_1 odometry.launch")


	- to see the tf tree , execute "rqt_tf_tree" while the node is still running. You can also see graphically the position of the car with rviz. The fixed frame is called "world" while the car frame is called "car"
	- to change the parameters execute "rosrun rqt_reconfigure rqt_reconfigure" is a separeted terminal
	- to see the odometry published on the topic. on a separate terminal source the environment and execute "rostopic echo odom_topic"

-------------------------------------------------------------------------------------------

Extra information:

The car frame is placed in the center between the rear wheels; the baseline is oriented along the y axis, while the d axis is oriented along x .

We noticed that with both models the car follows a circle.

Structure of the code in odometry.cpp:

-definition of the class Odometry, which is used to keep track of last pose (x,y,theta) and of the odometry type ( 0 = differential, 1 = Ackerman); it contains:

	-the methods for the computation of the differential drive odometry and the
	Ackerman odometry, which send also the transformations to the tf tree.
	-the param_callback method, invoked when one of the parameters is changed
	-the callback method, used to select the odometry, compute the new position and 	publish the result on the odom_topic

-definition of the main function, in which are used the message filters to declare the subscribers to speedL_stamped, steer_stamped and speedR_stamped (used to read the data). The dynamic reconfiguration is also set and the node spins at maximum allowable velocity.

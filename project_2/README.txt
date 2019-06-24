
HOMEWORK PROJECT OF ROBOTICS - A.A. 2018/2019

Members :

Barbera 	Chiara 		-- person code : 10691187 - ID : 920938
Fiorentino 	Alessio 	-- person code : 10495770 - ID : 920488
Francavilla	Simone 		-- person code : 10532021 - ID : 920142



-------------------------------------------------------------------------------------------


Inside the archive:

--> project_2: the package of the project, that contains:
	
	- src folder: the source code complementary_filter.cpp , complementary_filter_ros.cpp (used to handle the IMU data) and the odometry.cpp file, that computes the odometry

	- params folder: contains the navsat_transform_param.yaml file that is used by the navsat_transformation_node 
	
	- launch folder: contains the launch file; this one is used to run the ekf_node (that computes the localization with the EKF filter) and the navsat transformation node (that puts 				 together the imu data and the odometry)

	- include folder: contains the headers of the complementary filters source code
	

--> this README.txt file


In order to add the IMU data: 
1) the node odometry publishes the odometry on /odom_topic; 
2) the imu_tools subscribes to /swiftnav/rear/imu to get the imu data from the bag, adds the orientation and publishes the result on the /swiftnav/rear/imu/data;
3) the ekf node takes the data from /odom_topic and /swiftnav/rear/imu/data, puts them together and publishes the result on /odom_topic/filtered;

In order to add the GPS data:
1) the navsat subscribes to /swiftnav/rear/imu/data and swiftnav/rear/gps and /odom_topic/filtered;
2) the navsat computes the localization and publishes the final result on the /swiftnav/rear/gps/filtered;

Both ekf_node and navsat used other topics to publish data by default, but they have been remapped in the launch file.

-------------------------------------------------------------------------------------------

In the launch file are set the parameters of the ekf_node; between these, the imu0 and odom0 represent the topic from which the node takes the input data (/swiftnav/rear/imu/data and odom_topic respectively) and they are configured to abilitate only the x,y,z, roll, pitch, yaw, vx, vy, vz (for the odom0) and roll, pitch, yaw first derivative or roll and pitch and second derivative of x,y and z (for imu0). 

In the navsat_transform_param file, instead is set the frequency of spinning (30 Hz) , the magnetic_declination_radians (that depends on the geographical position); the yaw_offset, set to 180, has been found through the GPS raw data.

-------------------------------------------------------------------------------------------

Instructions:

How to execute the code :
	- run the bag
	- in a separate terminal, run the launch file ("roslaunch project_2 launcher.launch")

	- while the nodes are still running, you can also see graphically the position of the car with mapviz: open mapviz and add navsat in order to see the data on 
	  /swiftnav/rear/gps/filtered topic.
	- to see the odometry data corrected with the imu data execute "rostopic echo /odom_topic/filtered".
	- to see the gps published on the topic. on a separate terminal source the environment and execute "rostopic echo /swiftnav/rear/gps/filtered".

-------------------------------------------------------------------------------------------

Extra information:

The car frame is placed in the center between the rear wheels; the baseline is oriented along the y axis, while the d axis is oriented along x.
We noticed that the car follows a circle.

Structure of the code in odometry.cpp:

-definition of the class Odometry, which is used to keep track of last pose (x,y,theta); it contains:

	-the methods for the computation of the Ackerman odometry
	-the callback method, used to select the odometry, compute the new position and publish the result on the odom_topic

-definition of the main function, in which is created a subscriber to /speedsteer to read the information about the steering angle and the velocity used to compute the Ackerman odometry; here the imu_tools are declared too.






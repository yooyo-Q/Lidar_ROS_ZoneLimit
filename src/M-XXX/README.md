ROS driver for M-XXX laser.

User Guide:
	1. move M-XXX file into ros workspace
	2. run catkin_make in terminal in ros workspace
	3. modify host_ip and port num for your laser.
	4. set your Ethernet IPv4 address to 192.168.1.xxx (different than your host_ip and Netmask to 255.255.255.0
	5. roslaunch mxxx MXXX.launch or roslaunch Mxxx MXXX_display.launch to start Mxxx ros node.


ROS topic: sensor_msgs/LaserScan -> "/scan"

TF frame: /laser (default)

ROS Service: 
	1. "/mxxx/disconnect_laser_srv"
		* disconnect laser, require restart the node to reconnect laser.

	2. "/mxxx/start_laser_srv"
		* start receiving continous sensor_msgs/LaserScan.

	3. "/mxxx/stop_laser_srv"
		* stop receiving continous sensor_msgs/LaserScan.
		* example: rosservice call /Mxxx/stop_laser_srv

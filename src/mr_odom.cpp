#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

using namespace std;

class ROS_SUB {
	public:
		ROS_SUB();
		void topic_cb( sensor_msgs::Joy data);
	
	private:
		ros::NodeHandle _nh;
		//Subscriber object
		ros::Subscriber cmd_vel_sub;
		ros::Publisher odom_pub;
		geometry_msgs::Twist _cmd_vel;

};

ROS_SUB::ROS_SUB() {
	//Initialize a subscriber:
	//	Input: 	topic name: /numbers
	//			queue:	1
	//			Callback function
	//			Object context: the value of data members
	cmd_vel__sub = _nh.subscribe("/cmd_vel", 1, &ROS_SUB::topic_cb, this);
	odom_pub = _nh.advertise<geometry_msgs::Twist>("/odom", 1);


}

void ROS_SUB::topic_cb( geometry_msgs::Twist cmd_vel) {

	_cmd_vel = cmd_vel;


}

void odometry(){
	ros::Rate rate(100);

	while(ros::ok()){

		

		rate.sleep();
	}
}


int main( int argc, char** argv ) {

	//Init the ros node with  name
	ros::init(argc, argv, "joystick_sub");

	//Create the ROS_SUB class object
	ROS_SUB rs;

	ros::spin(); 
	return 0;
}
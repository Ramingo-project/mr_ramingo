#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"


using namespace std;

class ROS_SUB {
	public:
		ROS_SUB();
		void topic_cb( sensor_msgs::Joy data);
		void joystick();

	private:
		ros::NodeHandle _nh;
		//Subscriber and Publisher object
		ros::Subscriber joystick_sub;
		ros::Publisher joystick_pub;
		float x;
		float z;
};

ROS_SUB::ROS_SUB() {
	//Initialize a subscriber and a publisher
	joystick_sub = _nh.subscribe("/joy", 1, &ROS_SUB::topic_cb, this);
	joystick_pub = _nh.advertise<geometry_msgs::Twist>("/dynamixel_workbench/cmd_vel", 10);
	boost::thread loop_t(&ROS_SUB::joystick, this); 
}

void ROS_SUB::topic_cb( sensor_msgs::Joy data) {
    x = data.axes[1];
    z = data.axes[2];
}

void ROS_SUB::joystick(){

	ros::Rate rate(1000);
	geometry_msgs::Twist cmd;
	while(ros::ok()){
		cmd.linear.x = x;
		cmd.angular.z = z;
		joystick_pub.publish(cmd);
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
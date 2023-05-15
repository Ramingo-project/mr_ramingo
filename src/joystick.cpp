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
		ros::Subscriber joystick_sub;
		ros::Publisher joystick_pub;
        float acc;
        float steer;

};

ROS_SUB::ROS_SUB() {
	//Initialize a subscriber:
	//	Input: 	topic name: /numbers
	//			queue:	1
	//			Callback function
	//			Object context: the value of data members
	joystick_sub = _nh.subscribe("/joy", 1, &ROS_SUB::topic_cb, this);
	joystick_pub = _nh.advertise<geometry_msgs::Twist>("/mr_ramingo/cmd_vel", 10);

}

void ROS_SUB::topic_cb( sensor_msgs::Joy data) {
	geometry_msgs::Twist cmd;
    cmd.linear.x = data.axes[1];
    cmd.angular.z = data.axes[2];
	joystick_pub.publish(cmd);


}


int main( int argc, char** argv ) {

	//Init the ros node with  name
	ros::init(argc, argv, "joystick_sub");

	//Create the ROS_SUB class object
	ROS_SUB rs;

	ros::spin(); 
	return 0;
}
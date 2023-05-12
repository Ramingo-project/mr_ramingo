#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#define r_wheel 0.033 

using namespace std;

class ROS_SUB {
	public:
		ROS_SUB();
		void topic_cb( geometry_msgs::Twist data);
		void odometry();
		
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
	cmd_vel_sub = _nh.subscribe("/cmd_vel", 1, &ROS_SUB::topic_cb, this);
	odom_pub = _nh.advertise<nav_msgs::Odometry>("/mr_odom", 50);
	boost::thread loop_t(&ROS_SUB::odometry, this);  //thread for the filter
}

void ROS_SUB::topic_cb( geometry_msgs::Twist cmd_vel) {

	_cmd_vel = cmd_vel;

}

void ROS_SUB::odometry(){

    tf::TransformBroadcaster odom_broadcaster;
	tf::Transform transform;
	tf::Quaternion q;
   
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
  
    double vx = 0.0;
    double vth = 0.0;
   
    ros::Rate r(1000);
    while(ros::ok()){
   
		//ros::spinOnce();               // check for incoming messages
		vx = _cmd_vel.linear.x;
		vth = _cmd_vel.angular.z;
		
		//compute odometry in a typical way given the velocities of the robot
		x += (vx * cos(th))*0.001;
		y += (vx * sin(th))*0.001;
		th += vth *0.001;		
	
		//first, we'll publish the transform over tf
		transform.setOrigin(tf::Vector3(x, y, 0.0));
        q.setRPY(0, 0, th);
		transform.setRotation(q);

		//send the transform
		odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mr_odom", "base_link"));
	
		//next, we'll publish the odometry message over ROS
		/*nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "mr_odom";
	
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation.x = transform.getRotation().x();
		odom.pose.pose.orientation.y = transform.getRotation().y();
		odom.pose.pose.orientation.z = transform.getRotation().z();
		odom.pose.pose.orientation.w = transform.getRotation().w();
	
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = vth;
	
		//publish the message
		odom_pub.publish(odom);*/
	
		r.sleep();
   }
}




int main( int argc, char** argv ) {

	//Init the ros node with  name
	ros::init(argc, argv, "odometry_publisher");

	//Create the ROS_SUB class object
	ROS_SUB rs;

	ros::spin(); 
	return 0;
}
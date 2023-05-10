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
   
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
  
    double vx = 0.1;
    double vy = 0.0;
    double vth = 0.1;
   
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
	last_time = ros::Time::now();
   
    ros::Rate r(1.0);
    while(ros::ok()){
   
		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();
		vx = _cmd_vel.linear.x*r_wheel;
		vth = _cmd_vel.angular.z;
		
		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th)) * dt;
		double delta_y = (vx * sin(th)) * dt;
		double delta_th = vth * dt;
	
		x += delta_x;
		y += delta_y;
		th += delta_th;
	
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
	
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
	
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
	
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
	
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
	
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
	
		//publish the message
		odom_pub.publish(odom);
	
		last_time = current_time;
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
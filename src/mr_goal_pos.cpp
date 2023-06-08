#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "boost/thread.hpp"
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main( int argc, char** argv ) {
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle nh;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(1.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    float k=0.3;
    bool detected = false;
    while(!detected){
        try{
            listener.waitForTransform("base_link", "aruco_marker_frame", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("base_link", "aruco_marker_frame", ros::Time(0), transform);
            goal.target_pose.pose.position.x = transform.getOrigin().x()-0.3;
            goal.target_pose.pose.position.y = transform.getOrigin().y()-0.3;
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = transform.getRotation().z();
            goal.target_pose.pose.orientation.w = 1.0;
            ac.sendGoal(goal);
            detected =true;

        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
		    ros::Duration(0.2).sleep();
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 1*k;
            goal.target_pose.pose.position.y = ((rand()%3)-1)*k/2;
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
            ac.sendGoal(goal);
            detected =false;
        }
    }  

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Ramingo, arrived in marker position");
    else
        ROS_INFO("The base failed to move");

    return 0;
}
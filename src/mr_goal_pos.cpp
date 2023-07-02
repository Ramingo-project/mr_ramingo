#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

//Declaring a new SimpleActionClient with action of move_base_msgs::MoveBaseAction 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main( int argc, char** argv ) {

    ros::init(argc, argv, "move_base_client");

    //Initiating move_base client
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(1.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Declaring move base goal
    move_base_msgs::MoveBaseGoal goal;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Transform tf; 
    tf::TransformBroadcaster br;

    float k = 0.3;
    bool detected = false;



    while(!detected && ros::ok()){

        // Setting target frame id and time in the goal action
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        //If the marker isn't detected do random movements along x and y axes
        goal.target_pose.pose.position.x = 1*k;
        goal.target_pose.pose.position.y = ((rand()%3)-1)*k/2;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        //ROS_INFO("WHILE");
        try{
            cout<<"TRY BEGUN!"<<endl;
            listener.waitForTransform("base_link", "aruco_marker_frame", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("base_link", "aruco_marker_frame", ros::Time(0), transform);
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.pose.position.x = transform.getOrigin().x() -0.5;
            goal.target_pose.pose.position.y = transform.getOrigin().y() ;
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
            ac.cancelAllGoals();
            ROS_INFO("Marker detected!");
            ros::Duration(3.0);

            if(transform.getOrigin().x()<1.5){
                detected = true; 
                ros::Duration(0.2).sleep();
            }
            
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
		    ros::Duration(0.2).sleep();
            ROS_INFO("CATCH");
        }
        //Send the goal
        ac.sendGoal(goal);

    }  

    //ROS_INFO("research aruco_marker_frame");
    // Retreive the marker position again and set the right goal position 
    ROS_INFO("Research ok");

    ROS_INFO_STREAM(" Transform: " << 
            
            transform.getOrigin().x() << ", " << 
            transform.getOrigin().y() << ", " <<
            transform.getOrigin().z() << ", " << 
            transform.getRotation().x() << ", " << 
            transform.getRotation().y() << ", " << 
            transform.getRotation().z() << ", " << 
            transform.getRotation().w()
            );

    
    //ros::Duration(0.2);
    //ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Ramingo, arrived in marker position!");
    else
        ROS_INFO("The base failed to move!");

    return 0;
}
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;

//Declaring a new SimpleActionClient with action of
move_base_msgs::MoveBaseAction
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
    float k = 0.3;
    bool detected = false;


    while(!detected){

        // Setting target frame id and time in the goal action
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        //If the marker isn't detected do random movements along x and y axes
        goal.target_pose.pose.position.x = 1*k;
        goal.target_pose.pose.position.y = ((rand()%3)-1)*k/2;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        try{
            listener.waitForTransform("base_link", "aruco_marker_frame", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("base_link", "aruco_marker_frame", ros::Time(0), transform);
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
            detected = true;
            ros::Duration(2.0);
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
		    ros::Duration(0.2).sleep();
        }
        //Send the goal
        ac.sendGoal(goal);
    }  

    ROS_INFO("research aruco_marker_frame");

    // Retreive the marker position again and set the right goal position 
    listener.waitForTransform("base_link", "aruco_marker_frame", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_link", "aruco_marker_frame", ros::Time(0), transform);
    goal.target_pose.pose.position.x = transform.getOrigin().x();
    goal.target_pose.pose.position.y = transform.getOrigin().y();
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = transform.getRotation().z();
    goal.target_pose.pose.orientation.w = 1.0;
    ac.sendGoal(goal);
    
    ros::Duration(1.0);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Ramingo, arrived in marker position");
    else
        ROS_INFO("The base failed to move");

    return 0;
}
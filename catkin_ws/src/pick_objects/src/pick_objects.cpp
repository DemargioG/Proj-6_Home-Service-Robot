#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <vector>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

   // Node Handle
   ros::NodeHandle n;


  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  //move_base_msgs::MoveBaseFeedback feedBack;
  

  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 5.0;
  goal.target_pose.pose.position.y = -5.0;
  goal.target_pose.pose.orientation.w = 1.0; // 1.0575

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {

    //ros::Subscriber odom_sub = n.subscribe("/odom",1, odom_callback);    

    ROS_INFO("Hooray, we have arrived at the Pick Up destination.");
    

  // Pause for 5 seconds after making it to the pickup destiation.
    ros::Duration(5.0).sleep();
  }
  else
  {
    ROS_INFO("The base failed to make it to the Pick up point.");

  // Pause for 5 seconds 
    ros::Duration(5.0).sleep();
  }
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.5;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("We have arrived at the Drop Off destination.");
  // Pause for 5 seconds after making it to the pickup destiation.
    ros::Duration(5.0).sleep();
  }
  else
  {
    ROS_INFO("The base failed to move arrive at the Drop off point.");
  // Pause for 5 seconds after making it to the pickup destiation.
    ros::Duration(5.0).sleep();
  }


  return 0;
}

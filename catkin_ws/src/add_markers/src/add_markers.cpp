#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <vector>

///***********TEST SWITCH**********///
///Toggle for add_marker simulation///
bool test_add_marker;

// Create a global variable to store the position.
geometry_msgs::Point pose;

//Make the Publisher and Marker Global 
visualization_msgs::Marker marker;
ros::Publisher marker_pub;

// Create phse controling variables
bool pick_up = true;
bool drop_off = false;

// Set a Tolerance threshold incase the robot is not perfectly at the location.
double tolerance = 0.15;

// Initalize the Pick up point and the Drop off point. (C++ 98)
std::vector<double> goal_pickup (2,0.0);
std::vector<double> goal_dropoff(2,0.0);

void set_marker(visualization_msgs::Marker &marker, std::vector<double> goal)
{
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = goal[0];
  marker.pose.position.y = goal[1];
  marker.pose.position.z = 0.1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;


  // Set the scale of the marker -- .5x.5x.5 here means 1m on a side
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
}

void hide_marker(visualization_msgs::Marker &marker)
{
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.0;
}

void show_marker(visualization_msgs::Marker &marker)
{
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
}

void odom_callback(nav_msgs::Odometry odom)
{
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;

  //ROS_INFO("x: %f\n", pose.x);
  //ROS_INFO("y: %f\n", pose.y);

  // Sets the location of the marker depending on Goal.
  if (pick_up)
  {
    set_marker(marker, goal_pickup);
    show_marker(marker);
    ROS_WARN_ONCE("Heading to Pick Up");
  }
  if (drop_off)
  {
    set_marker(marker, goal_dropoff);
    ROS_WARN_ONCE("Heading to Drop Off");
  }

  // Publish the marker
  marker_pub.publish(marker);

  // Pick up Phase
  if (pick_up)
  { 
    if(fabs(goal_pickup[0] - pose.x) < tolerance && fabs(goal_pickup[1] - pose.y) < tolerance)
    {
       ROS_INFO("Picking up the Package.");
      // Wait 5 seconds at the Pick up Point.
      ros::Duration(5.0).sleep();

      //Change the Alpa to transparent thus hiding the marker.
      hide_marker(marker);

      // Change phase to Drop Off
      pick_up = false;
      drop_off = true;
    }
  }

  // Drop off Phase
  if (drop_off)
  {
     if(fabs(goal_dropoff[0] - pose.x) < tolerance && fabs(goal_dropoff[1] - pose.y) < tolerance)
    {
      //Change the Alpa to opaque thus showing the marker.
      show_marker(marker);

      // Change phase
      drop_off = false;
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::NodeHandle nh ("~");
  ros::Rate r(10);
  
  //Gets the "test_add_marker" parameter from the user. If true, it will run Test settings. If false or no inupt, it will run the regular settings.
  nh.getParam("test_add_marker", test_add_marker);
  
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom",10,odom_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Initalizing Goals (C++ 98)
  goal_pickup[0] = 5.0;
  goal_pickup[1] = -5.0;

  goal_dropoff[0] = 1.5;
  goal_dropoff[1] = 0.0;

    //visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;


    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  marker.lifetime = ros::Duration();

    // Check to make sure RViz is Subbed
  if (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  if (test_add_marker)
  {
    ROS_INFO("Test Mode Activated.");
    
    set_marker(marker, goal_pickup);
    show_marker(marker);
    marker_pub.publish(marker);

    ros::Duration(5.0).sleep();

    hide_marker(marker);
    marker_pub.publish(marker);

    ros::Duration(5.0).sleep();
    
    set_marker(marker, goal_dropoff);
    show_marker(marker);
    marker_pub.publish(marker);

    return 0;
  }
  else
  {
    ros::spin();
  }
}


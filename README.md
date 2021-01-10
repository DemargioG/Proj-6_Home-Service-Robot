# Proj-6_Home-Service-Robot

This is a project completed for the Robotics Software course of Udacity.

After spending a serious amount of time learning the core concepts of robotics programming in ROS, I had to put all my knowledge to the test.
I made a number of Packages, nodes, and Publishers / subscribers in this project to demonstrate what I learned. 

## To run locally:
You will need ROS (Meledoic Morenia is fairly popular).
Link to installation can be found here: http://wiki.ros.org/ROS/Installation

You will also need Catkin to help compile the packages.
A link to the ROS Wiki, which has a link to installation as well as helpful information, can be found below.
http://wiki.ros.org/catkin

##Packages and Files

-Shell Files
 Five shell files are in the inital folder. 
 The "home_service" file is the final collection of work and will run a program which reflects the goal of the project.
 The other for files are used to test smaller portions of the project.
 The "test_slam" file was used to to map the enviroment and objects in it.
 The "test_navigation" file used AMCL to see if the robot could navigate the enviroment when given a goal position.
 All at the same time, it would test the localizing ability of the robot. If the robot did not know where it was, it would fail to navigate.
 
 
-Add Markers
 The "add_markers" Package is used to create a virtual object in RViz to simulate the robot retrieving and delivering an object.
 The "add_markers" file is the only file which needs to be changed if you would like to test it individually. 
 To test the "add_markers" file, open the it in an editor and change the "test_add_marker" variable to True.
 After, save the file and catkin_make to compile the changes.
 
-Pick Objects
 The "pick_objects" Package is used to send goal locations to the robot.
 When used in conjunction with AMCL, the robot is able to navigate familiar enviroments.
 Should some aspects of the enviroment have changed, the robot will be able to find its way to the goal, assuming it is in an accessable location.
 
-My Robot
 The "my_robot" Package houses many of the involved features such as the launch files, world settings, and robot description.
 

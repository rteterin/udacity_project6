#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

const double PICKUP_X = 5;
const double PICKUP_Y = 5;

const double DROPOFF_X = 0;
const double DROPOFF_Y = 0;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Publisher status_publisher = n.advertise<std_msgs::String>("status", 1000);

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = PICKUP_X;
  goal.target_pose.pose.position.y = PICKUP_Y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Moving to the pickup zone");
  ac.sendGoal(goal);

  std_msgs::String status_message;

  status_message.data = "moving";
  status_publisher.publish(status_message);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Reached the pickup zone");
    status_message.data = "pickup";
    status_publisher.publish(status_message);
  } else {
    ROS_INFO("Failed to reach the pickup zone");
  }

  sleep(5);

  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = DROPOFF_X;
  goal.target_pose.pose.position.y = DROPOFF_Y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Moving to the drop-off zone");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Reached the drop-off zone");
    status_message.data = "drop-off";
    status_publisher.publish(status_message);
  } else {
    ROS_INFO("Failed to reach the drop-off zone");
  }

  sleep(5);

  return 0;
}

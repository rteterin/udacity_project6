#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ctime>

const double PICKUP_X = -3.5;
const double PICKUP_Y = 0.575;

double robot_x = 0;
double robot_y = 0;

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry)
{
    robot_x = odometry->pose.pose.position.x;
    robot_y = odometry->pose.pose.position.y;
}

void status_callback(std_msgs::String status)
{
    ROS_INFO("Received status: %s", status.data.c_str());

    if (status.data == "pickup")
    {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
    }
    else if (status.data == "drop-off")
    {
        marker.id = 1;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = robot_x;
        marker.pose.position.y = robot_y;
        marker.pose.position.z = 0;
        marker_pub.publish(marker);
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odometry_callback);
  ros::Subscriber status_sub = n.subscribe("/pick_objects/status", 1000, status_callback);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  // Publishing the marker at the pickup zone
  marker_pub.publish(marker);

  ros::spin();
}

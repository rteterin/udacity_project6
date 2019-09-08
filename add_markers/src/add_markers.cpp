#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ctime>

const double REQUIRED_DISTANCE = 0.3;

const double PICKUP_X = 5;
const double PICKUP_Y = 5;

const double DROPOFF_X = 0;
const double DROPOFF_Y = 0;

enum Status
{
    STATUS_MOVING_TO_PICKUP,
    STATUS_READY_TO_PICKUP,
    STATUS_PICKING_UP,
    STATUS_MOVING_TO_DROPOFF,
    STATUS_DONE
};

ros::Publisher marker_pub;

visualization_msgs::Marker marker;
Status status = STATUS_MOVING_TO_PICKUP;

std::time_t pickup_zone_arrival_time = 0;

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry)
{
    double position_x = odometry->pose.pose.position.x;
    double position_y = odometry->pose.pose.position.y;

    //ROS_INFO("%.4f %.4f", distance_from_pickup_2d, distance_from_dropoff_2d);

    switch (status) {
    case STATUS_MOVING_TO_PICKUP: {
        double distance_from_pickup_2d = sqrt(pow(position_x - PICKUP_X, 2) +
                                              pow(position_y - PICKUP_Y, 2));

        if (distance_from_pickup_2d < REQUIRED_DISTANCE) {
            ROS_INFO("Reached the pickup zone");
            status = STATUS_READY_TO_PICKUP;
            pickup_zone_arrival_time = std::time(0);
        }
        break;
    }
    case STATUS_READY_TO_PICKUP: {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        status = STATUS_PICKING_UP;

        break;
    }
    case STATUS_PICKING_UP: {
        if (std::time(0) - pickup_zone_arrival_time > 5)
        {
            ROS_INFO("Picked up the item");

            status = STATUS_MOVING_TO_DROPOFF;
        }
        break;
    }
    case STATUS_MOVING_TO_DROPOFF: {
        double distance_from_dropoff_2d = sqrt(pow(position_x - DROPOFF_X, 2) +
                                               pow(position_y - DROPOFF_Y, 2));

        if (distance_from_dropoff_2d < REQUIRED_DISTANCE)
        {
            ROS_INFO("Reached the drop-off zone");

            // Publishing the marker at the drop-off zone
            marker.id = 1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = DROPOFF_X;
            marker.pose.position.y = DROPOFF_Y;
            marker.pose.position.z = 0;
            marker_pub.publish(marker);

            status = STATUS_DONE;
        }
        break;
    }
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odometry_callback);

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

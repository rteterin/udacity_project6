## Used packages

### gmapping
Allows to generate a map of the environment from laser and odometry data using particle filter.
The map is then written to file with "map_server" ROS node.

## amcl
Allows to localize the robot within the generated map using adaptive Monte Carlo localization
approach.

### turtlebot_teleop
Allows to operate the robot with a keyboard for testing purposes.

### turtlebot_rviz_launchers
Used to launch RViz pre-configured for navigation/mapping.

### pick_objects
Creates and publishes goals for the robot to reach. The first goal published is the location
where the robot should pick up the object and the second one is where the robot should drop it off.
The package also publishes text messages containing current status of the robot.

### add_markers
Creates and publishes markers that can be displayed in RViz to simulate the object moved by the robot. First, a marker
is published at the pickup zone. When the robot reaches it, the marker is deleted,
meaning that the object has been picked up. Another marker is published when the robot reaches the drop-off zone.
This package subscribes to status messages from the "pick_objects" package to determine
when the robot reaches pickup and drop-off zones.

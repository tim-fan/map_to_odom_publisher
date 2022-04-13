# Map To Odom Publisher

## 1. Description
ROS package containing a single node for publishing `map`→`odom` transforms.

Subscribes to topic `current_pose` (type `nav_msgs/Odometry`) in `map` frame, 
looks up the current `odom`→`base_link` transform from tf, and computes and publishes
the current `map`→`odom` transform.

This is useful if you have a localisation system which publishes to a pose topic, but does not publish the `map` to `odom` transform as required by [REP 105](https://www.ros.org/reps/rep-0105.html).

## 2. Nodes
### 2.1 map_to_odom_publisher
Publishes the `map`→`odom` transform to `tf`, based on current pose topic.
### 2.1.1 Subscribed Topics
* `current_pose` ([`nav_msgs/Odometry`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html))
    * The current pose of the robot
* `tf`
    * listens for the `odom` → `base_link` transform
### 2.1.2 Published Topics
* `tf`
    * publishes the `map`→`odom` transform

### 2.1.3 Parameters
* `~odom_frame_id` (`string`, default: "odom")
    * The name of the odometric frame of the robot
* `~base_frame_id` (`string`, default: "base_link")
    * The name of of the robot frame
* `~global_frame_id` (`string`, default: "map")
    * The name of the global frame
* `~transform_tolerance` (`double`, default: 0.1)
    * Timeout to use for `tf` lookups

## Acknowledgements

The source for this node is a modified version of [fake_localization.cpp](https://github.com/ros-planning/navigation/blob/noetic-devel/fake_localization/fake_localization.cpp) from the [fake_localization](https://github.com/ros-planning/navigation/tree/noetic-devel/fake_localization) package, with most of the features stripped out.

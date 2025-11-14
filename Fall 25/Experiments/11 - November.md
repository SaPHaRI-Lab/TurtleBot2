
### November 14th, 2025

**Objective:** Obstacle Avoidance

**Experiment:**
- Config `move_base.launch.xml`
```
<!--

ROS navigation stack with velocity smoother and safety (reactive) controller

-->

<launch>
	<include file="$(find turtlebot_navigation)/launch/includes/velocity_smoother.launch.xml"/>
	<include file="$(find turtlebot_navigation)/launch/includes/safety_controller.launch.xml"/>
	
	<arg name="odom_frame_id" default="odom"/>
	<arg name="base_frame_id" default="base_footprint"/>
	<arg name="global_frame_id" default="map"/>
	<arg name="odom_topic" default="odom" />
	<arg name="laser_topic" default="scan" />
	<arg name="custom_param_file" default="$(find turtlebot_navigation)/param/dummy.yaml"/>

	<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
		<rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
		<rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
		<rosparam file="$(find turtlebot_navigation)/param/local_costmap_params.yaml" command="load" />
		<rosparam file="$(find turtlebot_navigation)/param/global_costmap_params.yaml" command="load" />
		<rosparam file="$(find turtlebot_navigation)/param/dwa_local_planner_params.yaml" command="load" />
		<rosparam file="$(find turtlebot_navigation)/param/move_base_params.yaml" command="load" />
		<rosparam file="$(find turtlebot_navigation)/param/global_planner_params.yaml" command="load" />
		<rosparam file="$(find turtlebot_navigation)/param/navfn_global_planner_params.yaml" command="load" />
		<!-- external params file that could be loaded into the move_base namespace -->
		<rosparam file="$(arg custom_param_file)" command="load" />

		<!-- reset frame_id parameters using user input data -->
		<param name="global_costmap/global_frame" value="$(arg global_frame_id)"/>
		<param name="global_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
		<param name="local_costmap/global_frame" value="$(arg odom_frame_id)"/>
		<param name="local_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
		<param name="DWAPlannerROS/global_frame_id" value="$(arg odom_frame_id)"/>
		
		<remap from="cmd_vel" to="cmd_vel_nav"/>
		<remap from="odom" to="$(arg odom_topic)"/>
		<remap from="scan" to="$(arg laser_topic)"/>
	</node>
</launch>
```
- Change in amcl file
	- Change this `<include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml">` into this `<include file="$(find turtlebot_navigation)/launch/includes/move_base2.launch.xml">`
- Create a python script
```
#!/usr/bin/env python3

import rospy

import math

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

  

SAFE_DISTANCE = 0.60 # slow down if below this

STOP_DISTANCE = 0.30 # stop if below this

  

class DynamicAvoidance:

def __init__(self):

rospy.init_node("dynamic_avoidance")

  

# Subscribers & publisher

rospy.Subscriber("/cmd_vel_nav", Twist, self.nav_callback)

rospy.Subscriber("/scan", LaserScan, self.scan_callback)

self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

  

self.min_front_distance = float('inf')

  

def scan_callback(self, msg):

# Extract front 60 degrees (30° left, 30° right)

total = len(msg.ranges)

width = int(total * 60.0 / 360.0) # number of samples for 60 degrees

half = width // 2

  

# Front center at index 0 (LaserScan starts at +X forward)

front_ranges = msg.ranges[:half] + msg.ranges[-half:]

  

# Filter bad values

clean = [r for r in front_ranges if not math.isnan(r) and r > 0.01 and r < msg.range_max]

  

if clean:

self.min_front_distance = min(clean)

else:

self.min_front_distance = float('inf')

  

rospy.loginfo_throttle(1.0, f"Front distance = {self.min_front_distance:.2f} m")

  

def nav_callback(self, msg):

safe = Twist()

  

d = self.min_front_distance

  

# HARD STOP (too close)

if d < STOP_DISTANCE:

rospy.loginfo_throttle(0.5, "HARD STOP")

safe.linear.x = 0.0

safe.angular.z = 0.0

  

# STRONG SLOW + STRONG TURN

elif d < 0.45:

rospy.loginfo_throttle(0.5, "STRONG AVOIDANCE")

safe.linear.x = 0.03

safe.angular.z = 0.6

  

# LIGHT SLOW + LIGHT TURN

elif d < SAFE_DISTANCE:

rospy.loginfo_throttle(0.5, "AVOIDANCE")

safe.linear.x = min(msg.linear.x, 0.08)

safe.angular.z = 0.3

  

# SAFE → follow move_base but clamp max speed

else:

safe = msg

safe.linear.x = min(msg.linear.x, 0.15) # <---- LIMIT SPEED

  

self.cmd_pub.publish(safe)

  

if __name__ == "__main__":

try:

DynamicAvoidance()

rospy.spin()

except rospy.ROSInterruptException:

pass
```

**Result:** Stop right in front of me
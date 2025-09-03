
### September 2nd, 2025

**Objective:** Continue checking Raspberry Pi

**Issue:**
- Raspberry Pi 2: Cannot teleoperation
	- Error: `ERROR: cannot launch node of type [turtlebot_teleop/turtlebot_teleop_key]: Cannot locate node of type [turtlebot_teleop_key] in package [turtlebot_teleop]. Make sure file exists in package path and permission is set to executable (chmod +x)`
	- Solution
		- `roscd turtlebot_teleop`
		- `cd scripts`
		- `chmod +x turtlebot_teleop_key`

**Result:**
- Raspberry Pi 2 can teleoperation (Keyboard)
	- Remote Control
		- one TurtleBot
		- two or more TurtleBots

### September 3rd, 2025

**Objective:** Continue checking Raspberry Pi

**Issues:**
- Raspberry Pi 4: Cannot update
	- After `sudo apt update`
	- Errors:
		- `W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>`
		- `W: Failed to fetch http://packages.ros.org/ros/ubuntu/dists/focal/InRelease  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>`
		- `W: Some index files failed to download. They have been ignored, or old ones used instead`
	- Solutions:
		- `sudo rm /etc/apt/sources.list.d/ros-latest.list`
		- `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
- Raspberry Pi 6
	- Needs to wipe out!

**Results:**
- Raspberry Pi 4 can teleoperation (Keyboard)
	- Remote Control
		- one TurtleBot
		- two or more TurtleBots
- Raspberry Pi 6 still reinstall
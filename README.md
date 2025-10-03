 **![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXcaUwHYFXEhbpmRKNrs-rXpKaGHXryAO5pWwI0KPXQLf8ZRtAFBG4KbLanLjZCD1Y8Nc-D5kFyhZKBfeRNSerG6eZFb7FVuGOkFQI4rZeSYTbf_icit9P8G553bO5lqjwXBgcHPJrQ7J-258zmAfjyJPk4d?key=s6kx8w9aAKgfuro0Mrp5kw)

# TurtleBot2

## Background

A TurtleBot is a low-cost, personal robot kit with open-source software. TurtleBots can move, map, and navigate environments, making them useful for mobile computer vision experiments. This project uses the TurtleBot2 with a Kobuki base. This version has a mobile base, a Kinect camera, and a sensor to see its surroundings to measure distances and help map its environment. And we have 7 of them.

### Goal
Create an automatically deployed system in which all 7 robots can communicate with one other and move together as a swarm. This system will be used to interact with people and investigate human-robot behavior.

### Components:
1. LiDAR Sensor **(replaces Xbox 360 Kinect)**
2. Speaker
3. Raspberry Pi **(replaces Jetson TK1)**
4. Kobuki Base
5. Robot Body
6. Battery

#### LiDAR Sensor
The LiDAR (Light Detection and Ranging) is a remote sensing technology that uses laser beams to measure distances and create detailed 3D maps of the environment. On the TurtleBots, the LiDAR sensor is mounted on top of the robot to provide a 360-degree range of measurements. We used a cardboard box to place it above the existing Kinect sensor.

![](https://m.media-amazon.com/images/I/515P0yY+uxL.jpg)

#### Raspberry Pi

The brain of the computer/robot, this is where all the calculations and decisions are made. In this research I use Raspberry Pi 4 Model B 8GB with Starter Kit (including case, fan, heat sync, charger, etc.).

Link from Amazon: [Raspberry Pi](https://www.amazon.com/CanaKit-Raspberry-4GB-Starter-Kit/dp/B08956GVXN/ref=sr_1_2_sspa?crid=1G4BEMXOX5NVU&dib=eyJ2IjoiMSJ9.mP4drOfyakW9P2E6ytjWi16gj2s3LrQBGuFeMtbTEh_hMvprgoi-t-zlc_pvFQPcF8E2O6AESj6Om7ZB9CrmRyba7rYttBJ7UUUxLrY5W3aubaUiDeB1AUSutuUgoQoQpqV06pSq0PIWx2_OFJHtnuSn5FbIeruK-kpbvBaTi5gvTZ_BBcB4iG9R8Kykr_gSTNvACwM9L2azfpDbIXxaEcS_7iFzVMzvMllRlt5t2BY.OT51DETFITwREkwJcV8xwrQ18fpsFBqTiz1Cksqw4_w&dib_tag=se&keywords=raspberry%2Bpi%2B4&qid=1720193522&sprefix=rasp%2Caps%2C98&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) 

## Getting Started

### Install Ubuntu Desktop on Raspberry Pi 4 8GB:

- Install Ubuntu Desktop on Raspberry Pi on Micro SD Card (32 GB preferred ____)
- Download the [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
- Open Raspberry Pi Imager
![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXfEk9S9muGok_69JQ9_wFfhn9Vb88JbpKIRL2qaasyb4bSmwAiA5fotHrRxcxwO5d0C-XGX7vVeHMuHdPsQ3t0X1FL_Hio2OFS62wKQki54P0nCACh-WYkXC1Mosvx1dzNd7V-5BUceXTIJep92OGYqMKJ9?key=s6kx8w9aAKgfuro0Mrp5kw)

- Choose device: Raspberry Pi 4
- Choose OS: Other general-purpose OS -> Ubuntu -> Ubuntu Server 20.04.05 LTS (64-bit)
- Choose Storage: your Micro SD Card
- Choose Next -> Edit setting
- The hostname, username, password, etc. can be anything. We configured wireless LAN and used:
	- Hostname: ubuntu
	- Username: SaPHaRI
	- Password: **7TurtleBotsX!** (with X being our assigned TurtleBot numbers)
	- Configured wireless LAN to our own lab network
    

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXdtskk92z3T32FkdkXlHTUhXhYmyUOzf7909V-DsNr4zZ1j7_h5aMhnyEpfKWwHSow_K2CAhBmuVZ99TTMqj08_ASnJqinstS5OMA39LVOL9En1b9SBI9He2VDz0X8eHLBhYO94lo33F1ZGT1mz1js8n4zx?key=s6kx8w9aAKgfuro0Mrp5kw)

- In Services: Enable SSH (Use password authentication)
![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXc7DQ4zxonYHdd9CRBJykDt-v-UH9utN-Fb8nmPX2rnh63F9CSGTbtXYgmT045s-fv093jcw13pHN6yNutD-gHWdjSCG8Wc4733WVAtUUZrfiDY_SzSbksuMU4ienj0OeGh6_kFj-wNpzdZU95Qco6qNm4?key=s6kx8w9aAKgfuro0Mrp5kw)
- In Options: Depends on you

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXd38wKhGEv3QKH1AZHkfQlUv25zBhKw72mxJQMFrHExRKOztEObG2c-e5WthZMxlwJpH2OUG-7psCbVr_SbEKNLU6soShJ8rZK7Tvji64948IDmLHP4ArEBZs1TTZ7imsUS8QKw0xxL7Ufas19hWuAtGDU?key=s6kx8w9aAKgfuro0Mrp5kw)

- Choose yes: Your files in Micro SD Card will erase and install Ubuntu Server instead
- Wait until it finishes
- Insert Micro SD Card into Raspberry Pi
- Connect Keyboard, Mouse, External Monitor (Micro HDMI to Regular HDMI) before powering up the Raspberry Pi
- After powering up the Raspberry Pi, it should boot up the Ubuntu Server
- When it shows cloud-init … finish at..
    

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXdO9HAlX4wqU6ryVqCsdmnO74PqWFx0kfpd5OCYwJ7U5ySs1n9I0T__shiJeI551zsIoOU56mbmS2VIfblnFiLompRehPKmNPk7E2F_vl_D4e7mUw2as_bbub3yLSKsV1KQBNqkykOJStCzuCf-FX1bc7wj?key=s6kx8w9aAKgfuro0Mrp5kw)
- `Ctrl + Alt + F2`
- Login with your username and password that you set up in the beginning

- Update & Upgrade  
  - `sudo apt update`  
  - `sudo apt upgrade`  
  - `sudo apt update && sudo apt upgrade -y` (Recommended)

- Install Ubuntu Desktop  
  - `sudo apt install ubuntu-desktop -y`  
  - `startx` (To enter Ubuntu desktop)

### SSH:
- SSH with `ssh <username>@<ip>`
	- Our case it is `ssh SaPHaRI@192.168.0.___`
### Install ROS Noetic:
- Links “[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)”
- Install ROS Noetic Desktop Full version
	- Setup your sources.list
		- `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
	- Setup your keys
		- `sudo apt install curl`
		- `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
	- Installation
		- `sudo apt update && sudo apt upgrade -y`
		- `sudo apt install ros-noetic-desktop-full -y`
	- Environment setup
		- `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
		- `source ~/.bashrc`
		- In the environment options you can use zshrc or bash, I use bash
	- Dependencies for building packages
		- `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y`
	- Initialize rosdep
		- `sudo apt install python3-rosdep`
		- `sudo rosdep init`
		- `rosdep update`

You can skip the next section by cloning the end result of `kobuki_ws` [here](https://github.com/SaPHaRI-Lab/Turtlebot2-kobuki_ws). Else, follow the next section to do it manually.

### Setting up the workspace
- Open your terminal
	- Make your workspace with `mkdir kobuki_ws`
	- `cd kobuki_ws/`
	- `mkdir src` 
	- `catkin_make` (builds the workspace)
- Run `cd src/` then clone these repositories in the src:
	- `git clone https://github.com/yujinrobot/kobuki.git`
	- `git clone https://github.com/turtlebot/turtlebot.git`
	- `git clone https://github.com/yujinrobot/yujin_ocs.git`
	- `git clone https://github.com/turtlebot/turtlebot_msgs.git`
	- `git clone https://github.com/turtlebot/turtlebot_apps.git`
	- `git clone https://github.com/turtlebot/turtlebot_simulator.git`
	- `git clone https://github.com/OpenKinect/libfreenect.git`
	- `git clone https://github.com/ros-drivers/freenect_stack.git`
	- `git clone https://github.com/Slamtec/rplidar_ros.git`
- Install:
	- `sudo apt install ros-noetic-rtabmap-ros -y`
	- `sudo apt-get install ros-noetic-pointcloud-to-laserscan -y`
	- `sudo apt install liborocos-kdl-dev`
- Configuration 
	- Head to `kobuki_ws/src/yujin_ocs` then delete the all files except for
		- `yocs_cmd_vel_mux`
		- `yocs_controllers`
		- `yocs_velocity_smoother`
- Run rosdep (for ROS dependencies) command  
	- `cd kobuki_ws/`
	- `rosdep install --from-paths src --ignore-src -r -y`
	- `rosdep update`
	- `catkin_make`
- Source the workspace with `source devel/setup.bash`
- Test connection to the TurtleBot  
	- `lsusb` (to check if the TurtleBot is connected or not)  
	- `roslaunch turtlebot_bringup minimal.launch`
- If there is an error related to python, try:
	- `python3 --version`
	- `sudo ln -s /usr/bin/python3 /usr/bin/python`
- After turning on the Raspberry Pi again might have an error  
	- `catkin_make`
	- `source devel/setup.bash`
- Run the desired command again
	- Use this command to avoid running `source devel/setup.bash` every time after `catkin_make`  
	- `echo "source your_ws/devel/setup.bash" >> ~/.bashrc`
	- `source ~/.bashrc`

### Piloting the TurtleBot: Keyboard
In one terminal, run `roslaunch turtlebot_bringup minimal.launch`. In another, run `roslaunch turtlebot_teleop keyboard_teleop.launch` and follow the instructions of the file.

### Piloting the TurtleBot: Controller
- Pairing via CLI. **If pairing with GUI, skip this segment.** 
	- Get the MAC address of your controller. We used this pink [Xbox One S controller](https://www.amazon.com/Xbox-Core-Wireless-Controller-Deep-Controllers/dp/B09VV5LJS1/ref=sr_1_3?crid=WRHIS5YGVSQJ&dib=eyJ2IjoiMSJ9.esGG7pafXCEk_c9mSITHVcBlrZYuOm3xS4oFy_YYwoGmMOfyrE4rAzS9ZDX1dwrBgxneSjdiEvMoHCTkugBokpJbbrazNr7NgUStsEDX5OJZdLL83LrRADo4327KV-TBLhTweCbSpkpB_2SFOnkOgUGIrIbLQR7ozhDolmtQAWO6LyOpOrUsCpLlW79fBA4A0u8L53py4xlT4YrD4Ycj6bOefVGDzH04NlYc6l7-dP4.QCs3oWDtY4wYO0JLsHYXcujmf8vWqxxl57cJtEEkgTs&dib_tag=se&keywords=xbox%2Bone%2Bcontroller&qid=1728407453&s=videogames&sprefix=xbox%2Bone%2Bcontroller%2Cvideogames%2C89&sr=1-3&th=1)
	- The MAC address is not needed.
	- On a separate device (I used a Mac), connect the controller via the Bluetooth menu
	- In a terminal, run `system_profiler SPBluetoothDataType` to grab the MAC address of the controller
	- Unpair and forget the controller so we can connect it to the desktop
- To pair on the ROS master machine (in this case, our desktop with Ubuntu 20.04)
	- Since the Xbox One S is a newer controller, run `sudo apt remove xboxdrv` which removes the legacy driver
	- On the desktop, after running `bluetoothctl`, use:
		- `scan on` – ensure the same MAC address pops up
		- `pair <MAC address>` – might not work, which is fine
		- `connect <MAC address` – should work
		- `trust <MAC address>` – should work
	- To verify the controller input config, `sudo apt-get install jstest-gtk` and run the program
		- Run `sudo apt-get install evtest && sudo evtest`, then select the controller from the list to test inputs
		- Pay attention to the event IDs when pressing the buttons, these are the mappings 
- Piloting the Turtlebot2
	- Similar process to launching the `keyboard_teleop` above
	- After launch `minimal.launch`, in another terminal launch `roslaunch turtlebot_teleop xbox360_teleop`
	- Hold the "Y" button and move the leftmost joystick to move the Turtlebot.
	- Woohoo!

### Using the Kinect camera  
- Open the terminal
	- `sudo apt update && sudo apt upgrade -y` (Recommended)  
- Install the dependencies in your workspace
	- `cd kobuki_ws/src/`  
	- `sudo apt install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev -y`  
	- `git clone https://github.com/OpenKinect/libfreenect.git`
- Make and install 
	- `cd libfreenect`  
	- `mkdir build`  
	- `cd build`  
	- `cmake -L ..`  
	- `make`  
	- `sudo make install`  
	- `sudo ldconfig /usr/local/lib64/`
- To use Kinect without sudoing every time  
	- `sudo adduser $USER video`  
	- `sudo adduser $USER plugdev`
- Add some device rules  
	- `sudo nano /etc/udev/rules.d/51-kinect.rules`
	- Paste the following into above command and save:
```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```

- Need to generate audio drivers for support in freenect folder
	- `cd ..`
	- `python3 src/fwfetcher.py`
- Copy to a specific location (without this command still work)
	- `sudo cp src/audios.bin /usr/local/share/libfreenect`
- Open the Kinect camera (Run command in libfreenect folder)
	- `freenect-micview` (Checking the audio)
	- `Ctrl + c` 
	- `freenect-glview` (Open the camera)
	- `freenect-camtest` (To test camera)

### Toggling the Camera with RViz
- `cd kobuki_ws/src`
- `git clone https://github.com/ros-drivers/freenect_stack.git`
- Get back to workspace
	- `cd kobuki_ws`
	- `cd ..` (Recommended)
	- `catkin_make`
	- `source devel/setup.bash`
	- `roslaunch freenect_launch freenect.launch depth_registration:=true`
	- `rviz`
- Global Options
- Set fixed frame to `camera_link`

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXcPv0RzYZM7lzzI1F9DROsNuaTHmjLlQAirSWPPS7z8yzQSPFgIrgPOIIl48zmf9XL8s6btg25xdVq6tP6Hr0gi7y6FEV_ReO4zjrVT0XNyKMw691bOJjgKyFg31QEXHIeUrOJz57HVogMZjfnl-34EMM4?key=s6kx8w9aAKgfuro0Mrp5kw)

- Add pointcloud2
- Set the topic -> /camera/depth_registered/points
![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXdsWZlzsUEnnLTaYFQ15p6EzN10owXgu0eOxB7GGbcdLU8TPaTNxYsUPr_A9eScc_nHrKBH4fOxCdBuyopa-SLmkIZ0Hdzk_owakUd99b5Jmv2S9Mp7tGhu20OZw1UMB1YLYS2o8w245_ufA-GZp3oIuRsE?key=s6kx8w9aAKgfuro0Mrp5kw)

### Mapping (Toggle camera with RTAB-MAP and RViz)
- Open the terminal  
- Connect to the TurtleBot  
	- `roslaunch turtlebot_bringup minimal.launch`  
- Connect to the camera  
	- `roslaunch freenect_launch freenect.launch depth_registration:=true`  
- Install RTAB-MAP package  (if already install skip)
	- `sudo apt install ros-noetic-rtabmap-ros -y`  
- Open the RTAB-MAP  
	- `roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/depth_registered/image_raw rgb_topic:=/camera/rgb/image_raw camera_info_topic:=/camera/rgb/camera_info`  
- If facing any error on running the RTAB-MAP try this command  
	- `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib`  
	- `echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib' >> ~/.bashrc`  
	- `source ~/.bashrc`  
- Open the Rviz  
	- `rosrun rviz rviz`  
- Add option on Rviz  
	- Global options -> Set fixed frame -> `map`
	- Add RobotModel
	- Add TF
	- Add LaserScan
	- Map -> Topic -> `/rtabmap/cloud_map`
	- Odometry -> Topic -> `/rtabmap/odom`
	- Path -> Topic -> `/rtabmap/mapPath`
	- Add PointCloud2 -> Set topic -> `/rtabmap/cloud_map`
- Saving the map
	- Need to ensure correct topic: If map_saver doesn’t automatically save from the correct topic, need to specific the topic manually
		- Global Options: /map
		- Map: `/rtabmap/grid_prob_map`
	- Save map command
		- `rosrun map_server map_saver map:=/rtabmap/grid_prob_map -f /home/SaPHaRI/kobuki_ws/src/1stmap`
		- `map_saver map:=/rtabmap/grid_prob_map` is specific the topic manually
		- `/home/SaPHaRI/kobuki_ws/src/1stmap` is the path of the map
		- `1stmap` is name of the map

### Communication: 2 Terminals
- First terminal  
	- `roscore`  
- Second terminal  
	- `rostopic pub /test std_msgs/String "data: 'Hello from master'"`  
- Third terminal  
	- `rostopic echo /test`

### Communication: 2 Raspberry Pis
- First Raspberry Pi  
	- First Terminal
		- `roscore`  
	- Second Terminal  
		- `export ROS_MASTER_URI=http://<Hostname or IP Address>:11311`  
		- `export ROS_IP=<IP of Machine>`  
		- `rosrun beginner_tutorials talker.py`  
- Second Raspberry Pi
	- First Terminal
		- `export ROS_MASTER_URI=http://<Hostname or IP Address>:11311`  
		- `export ROS_IP=<IP of Machine>`  
		- `rosrun beginner_tutorials listener.py`

### Communication: 2+ Raspberry Pis
- Repeat the same procedure as the above
- Add more devices  
	- `export ROS_MASTER_URI=http://<Hostname or IP Address>:11311`  
	- `export ROS_IP=<IP of Machine>`  
	- `rosrun beginner_tutorials listener.py`

### Remote Control: 1 TurtleBot 
- Assume Rasp 1 is the master (talker) and Rasp 2 is a subscriber (listener). We'll control TurtleBot (Rasp 2) through Rasp 1  
- Rasp 1 (IP Address: 192.168.0.91)  
	- `export ROS_MASTER_URI=http://192.168.0.91:11311`  
	- `export ROS_IP=192.168.0.91`  
	- `roscore`
- Rasp 2 (IP Address: 192.168.0.190)  
	- `export ROS_MASTER_URI=http://192.168.0.91:11311`  
	- `export ROS_IP=192.168.0.190`  
	- `roslaunch turtlebot_bringup minimal.launch`  
- Rasp 1  
	- In another terminal:	`roslaunch turtlebot_teleop keyboard_teleop.launch`

### Remote Control: 2+ TurtleBots (simultaneous)
- Assumed Rasp1 is the master (talker) and Rasp 2, 3 are subscribers (listeners). Control TurtleBot (Rasp 2, 3) by using Rasp 1  
- Rasp 1 (IP Address: 192.168.0.91)
	- `export ROS_MASTER_URI=http://192.168.0.91:11311`  
	- `export ROS_IP=192.168.0.91`  
	- `roscore`  
	- `roslaunch turtlebot_teleop keyboard_teleop.launch`  
- Rasp 2 (IP Address: 192.168.0.190)  
	- `export ROS_MASTER_URI=http://192.168.0.91:11311`  
	- `export ROS_IP=192.168.0.190`  
	- `roslaunch turtlebot_bringup minimal.launch __ns:=<turtlebotname>` (need this command because to control 2 TurtleBots at the same time, you need different node names)
	- In another terminal  
		- `rosrun topic_tools relay <original topic name> <new topic name because of namespace change>`
		- `/cmd_vel_mux/input/teleop`  
		- `/turtlename/cmd_vel_mux/input/teleop`
- Rasp 3 (IP Address: 192.168.0.8)  
	- `export ROS_MASTER_URI=http://192.168.0.91:11311`  
	- `export ROS_IP=192.168.0.8`  
	- `roslaunch turtlebot_bringup minimal.launch __ns:=<turtlebotname>` (need this command because to control 2 TurtleBots at the same time, you need different node names)
	- In another terminal  
		- `rostopic list`  
		- `rosrun topic_tools relay <original topic name> <new topic name because of namespace change>`  
		- `/cmd_vel_mux/input/teleop`  
		- `/turtlename/cmd_vel_mux/input/teleop`
- Tips
	- If you don't want to indicate the name of TurtleBot every single time
		- `nano ~/.bashrc`
		- `export ROS_NAMESPACE=turtlebotX`
		- turtlebotX can be change

### When you're given a computer with Ubuntu but you don't know the login to the only existing user so you need to add a new one instead to move forward
Add a User via Recovery Mode
- Reboot the System
	- Reboot the system, and as it boots up, hold the **Shift** key to access the **GRUB** boot menu
- Select Recovery Mode
	- In the GRUB menu, select the **Advanced options for Ubuntu.** Then select the **Recovery mode** for your current kernel (the one ending in (recovery mode))
- Drop to Root Shell
	- Once in the recovery mode, select **Root - Drop to root shell prompt.** 
- Remount the Filesystem
	- `mount -o remount,rw /`
- Create a New User
	- `adduser newusername`
	- newusername = enter the desire name
	- Then follow the pormpts to set the password and other details for the new user
	- In other details
		- Full name, Room number, Phone number, etc. you can skip by press **enter** key
- Grant Sudo Privileges
	- `usermod -aG sudo newusername`
	- newusername = enter the name that you put in the create a new user part
- Reboot the System
	- `reboot`
	- Boots up in log in with the new user
**Username: SaPHaRI** 
**Password: 7TurtleBots!**

### Path Planning via SSH
- If you care for do the path planning on Raspberry Pi itself
	- Follow below steps just without ssh and run on the Raspberry Pi
- Assumed you have map
	- If no, go to Section: Save the Map above
- TurteBot
	- Turn on
- Raspberry Pi
	- Turn on
	- Connection via USB
		- TurtleBot
		- Kinect Camera
- Computer
	- Turn on
	- Open Terminal
	- Terminal 1 (Connect to TurtleBot)
		- `ssh SaPHaRI@192.168.0.53`
		- IP Address depends on what Raspberry Pi you use, in this case I use Raspberry Pi 5
		- `roslaunch turtlebot_bringup minimal.launch`
	- Terminal 2 (Freenect)
		- `ssh SaPHaRI@192.168.0.53`
		- `roslaunch freenect_launch freenect.launch depth_registration:=true`
	- Terminal 3 (RTABMAP)
		- `ssh SaPHaRI@192.168.0.53`
		- `roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/depth_registered/image_raw rgb_topic:=/camera/rgb/image_raw camera_info_topic:=/camera/rgb/camera_info`
	- Terminal 4 (AMCL file to initialize pose of TurtleBot)
		- `ssh SaPHaRI@192.168.0.53`
		- `roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/SaPHaRI/kobuki_ws/src/4thmap.yaml`
		- Don't forget to choose the map, in this case I chose `4thmap`
	- Terminal 5 (Rviz to do path planning)
		- `rviz`
		- Add option on Rviz
			- RobotModel
			- LaserScan -> Topic -> /scan
			- Map -> Topic -> /map
	- In Rviz
		- Click 2D Pose Estimate
			- To set initialize position on the robot
			- Set arrows go in the **same direction** with TurtleBot
		- Click 2D Nav Goal
			- Choose your Goal on the map


### Configuration file for AMCL
- Go to AMCL file
	- `cd kobuki_ws/src/turtlebot_apps/turtlebot_navigation/launch/`
	- `nano amcl_demo.launch`
	- Use this instead
```
<launch>
  <!-- Convert PointCloud to LaserScan -->
  <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
    <!-- Remap input from the Kinect's point cloud -->
    <remap from="cloud_in" to="/rtabmap/cloud_map"/>
    <param name="scan_time" value="0.033"/>  
    <param name="range_min" value="0.5"/>    
    <param name="range_max" value="5.0"/>    
    <param name="use_inf" value="true"/>     
    <param name="angle_min" value="-1.57"/>  
    <param name="angle_max" value="1.57"/>   
    <param name="target_frame" value="base_link"/>  
  </node>

  <!-- Map server -->
  <arg name="map_file" default="$(env TURTLEBOT_MAP_FILE)"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- AMCL -->
  <arg name="custom_amcl_launch_file" default="$(find turtlebot_navigation)/launch/includes/amcl/kinect_amcl.launch.xml"/>
  <arg name="initial_pose_x" default="0.0"/> 
  <arg name="initial_pose_y" default="0.0"/> 
  <arg name="initial_pose_a" default="0.0"/>
  <include file="$(arg custom_amcl_launch_file)">
    <arg name="initial_pose_x" value="$(arg initial_pose_x)"/>
    <arg name="initial_pose_y" value="$(arg initial_pose_y)"/>
    <arg name="initial_pose_a" value="$(arg initial_pose_a)"/>

    <!-- Add scan topic remapping here -->
    <arg name="scan_topic" default="/scan"/>
  </include>

  <!-- Move base -->
  <arg name="custom_param_file" default="$(find turtlebot_navigation)/param/kinect_costmap_params.yaml"/>
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml">
    <arg name="custom_param_file" value="$(arg custom_param_file)"/>
  </include>

</launch>
```
 - This AMCL file included move_base
	 - No need to launch `move_base.launch` anymore
 - Before launch this file you need to install pointcloud_to_laserscan
	 - `sudo apt-get install ros-noetic-pointcloud-to-laserscan -y`
- Then make sure the frame transformation (TF)
	- `sudo nano /opt/ros/noetic/lib/tf/view_frames`
	- Add `.decode('utf.8')` after `m = r.search(vstr)`
	- Should be like this `m = r.search(vstr.decode('utf.8'))`
- Explanation
	- Convert PointCloud to LaserScan
		- Due to Kinect, it doesn't produce a LaserScan directly; Instead, it produces a PointCloud2 like `/camera/depth/points`. You can convert the PointCloud2 to a LaserScan
		- But in this case I can `/rtabmap/cloud_map` because I use RTAB-MAP to help me map environment
	- Map server
		- To open the map file in the Rviz
	- AMCL
		- To initialize the pose of the robot
	- Move base
		- To enable the TurtleBot to move autonomously
- Tips: Make sure you set the topic in the Rviz same as in the AMCL file


### Receive the signal from Doorbell (Raspberry Pi)
- **Note: This is outdated. For the system we integrate with a mechanical doorbell opening system, see /Kaleb/Week 13.**
- Connection
![[RF Receiver MX-RM-5V Close up.png]]
- RF Receiver pins (Start from left) | Picture on the above
	- 1st: GND - Connect to a ground in Raspberry Pi pin 6th
	- 2nd: DATA (Signal) - Connect to a GPIO in Raspberry Pi pin 11th/13th
	- 3rd: ANT (Antenna) - Skip
	- 4th: 5V power - Connect to a 5V in Raspberry Pi pin 4th
-  Open terminal
	- `sudo apt-get install python3-pip`
	- `sudp pip3 install rpi-rf`
	- `nano rpi-rf_receive.py`
```
#!/usr/bin/env python3

import argparse
import signal
import sys
import time
import logging

from rpi_rf import RFDevice

rfdevice = None

# pylint: disable=unused-argument
def exithandler(signal, frame):
    rfdevice.cleanup()
    sys.exit(0)

logging.basicConfig(level=logging.INFO, datefmt='%Y-%m-%d %H:%M:%S',
                    format='%(asctime)-15s - [%(levelname)s] %(module)s: %(message)s', )
  
parser = argparse.ArgumentParser(description='Receives a decimal code via a 433/315MHz GPIO device')
parser.add_argument('-g', dest='gpio', type=int, default=27,
                    help="GPIO pin (Default: 27)")
args = parser.parse_args()
 
signal.signal(signal.SIGINT, exithandler)
rfdevice = RFDevice(args.gpio)
rfdevice.enable_rx()
timestamp = None
logging.info("Listening for codes on GPIO " + str(args.gpio))
while True:
    if rfdevice.rx_code_timestamp != timestamp:
        timestamp = rfdevice.rx_code_timestamp
        logging.info(str(rfdevice.rx_code) +
                     " [pulselength " + str(rfdevice.rx_pulselength) +
                     ", protocol " + str(rfdevice.rx_proto) + "]")
    time.sleep(0.01)
rfdevice.cleanup()
```
- After finished the setup try the code
	- `sudo python3 rpi-rf_receive.py`
- Press the doorbell button
	- receive the signal
![[Received RF signal.png]]

### Enable the TurtleBot to move autonomously by Doorbell
- Create necessary files
	- `cd kobuki_ws/src`
- Send goal file
	- `nano send_goal.py`
```
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def send_goal():
    rospy.init_node('send_goal')

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow time for the node to initialize

    goal = PoseStamped()

    # Set header information
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    # Set goal position (example values)
    goal.pose.position.x = 0.655
    goal.pose.position.y = -0.862
    goal.pose.position.z = 0.000

    # Set goal orientation (example quaternion)
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = -0.0753
    goal.pose.orientation.w = 0.658
    
    pub.publish(goal)
    rospy.loginfo("Goal sent to TurtleBot!")

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass
```
- Need to find the exact coordinate to config in the file (send_goal.py)
	- Find by do path planning, using 2D pose and 2D navigate
	- It will show in the 5th terminal that you lunched rviz
		- This coordinate is just to test the system
		- Setting Pose
			- Frame: map
			- -0.087 -0.152 0.181
		- Setting Goal
			- Frame: map
			- Position: 0.655 -0.862 0.000
			- Orientation: 0.000 0.000 -0.753 0.658
			- Angle: -1.704
- Combined send goal file and receive the signal file
	- `nano rpi-rd_receive+launch.py` (Just a file name, you can use other)
```
#!/usr/bin/env python3

import argparse
import signal
import sys
import time
import logging
import os  

from rpi_rf import RFDevice

rfdevice = None

# Handle exit signal
def exithandler(signal, frame):
    rfdevice.cleanup()
    sys.exit(0)

logging.basicConfig(level=logging.INFO, datefmt='%Y-%m-%d %H:%M:%S',
                    format='%(asctime)-15s - [%(levelname)s] %(module)s: %(message)s', )

# Argument parser for GPIO pin
parser = argparse.ArgumentParser(description='Receives a decimal code via a 433/315MHz GPIO device')
parser.add_argument('-g', dest='gpio', type=int, default=27,
                    help="GPIO pin (Default: 27)")
args = parser.parse_args()

# Setup exit handler
signal.signal(signal.SIGINT, exithandler)
rfdevice = RFDevice(args.gpio)
rfdevice.enable_rx()

timestamp = None
logging.info("Listening for codes on GPIO " + str(args.gpio))

# Main loop to listen for RF codes
while True:
    if rfdevice.rx_code_timestamp != timestamp:
        timestamp = rfdevice.rx_code_timestamp
        logging.info(str(rfdevice.rx_code) +
                     " [pulselength " + str(rfdevice.rx_pulselength) +
                     ", protocol " + str(rfdevice.rx_proto) + "]")
    time.sleep(0.01)

    # Trigger TurtleBot to move when an RF code is received
    if rfdevice.rx_code:
        os.system("bash -c 'source /opt/ros/noetic/setup.bash && source /home/SaPHaRI/kobuki_ws/devel/setup.bash && python3 /home/SaPHaRI/kobuki_ws/src/send_goal.py'")

rfdevice.cleanup()
```
- Test the system
	- Terminal 1:`roslaunch turtlebot_bringup minimal.launch`
	- Terminal 2: `roslaunch freenect_launch freenect.launch depth_registration:=true`
	- Terminal 3: `roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/depth_registered/image_raw rgb_topic:=/camera/rgb/image_raw camera_info_topic:=/camera/rgb/camera_info`
	- Terminal 4: `roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/SaPHaRI/kobuki_ws/src/map/4thmap.yaml`
	- Terminal 5: 
		- `sudo -python3 /home/SaPHaRI/kobuki_ws/src/rpi-rf_receive+launch.py`
		- or
		- `cd kobuki_ws/src`
		- `sudo python3 rpi-rf_receive+launch.py` 


### RPLIDAR Sensor (Integrate with mapping path)
- Install the package
	- `cd kobuki_ws/src`
	- `git clone https://github.com/Slamtec/rplidar_ros.git`
	- `cd ..`
	- `catkin_make`
- Config the `rplidar_a1.launch`
	- `cd kobuki_ws/src/rplidar_ros/launch`
	- `nano rplidar_a1.launch`
```
<launch>
  <!-- RPLIDAR Node -->
  <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode" output="screen">
    <param name="serial_port" type="string" value="/dev/ttyUSB0"/>
    <param name="serial_baudrate" type="int" value="115200"/>
    <param name="frame_id" type="string" value="base_laser"/>
    <param name="inverted" type="bool" value="true"/>
    <param name="angle_compensate" type="bool" value="true"/>
  </node>

  <!-- Static Transform Publisher -->
  <node name="static_transform_publisher" pkg="tf2_ros" type="static_transform_publisher" output="screen" args="0 0 0.6 0 1 0 0 base_link base_laser"/>

</launch>
```
- `roslaunch turtlebot_bringup minimal.launch`
- `roslaunch rplidar_ros rplidar_a1.launch`
- PS. If lidar sensor error with exit code 255, please check the USB port
	- `ls /dev/ttyUSB*`


### Configuration GMapping before do mapping
- `cd kobuki_ws/src/turtlebot_apps/turtlebot_navigation/launch`
- `nano gmapping_demo.launch`
```
<launch>
  <!-- RPLIDAR node should be launched separately, as you are already doing -->

  <!-- Gmapping -->
  <include file="$(find turtlebot_navigation)/launch/includes/gmapping/gmapping.launch.xml">
    <arg name="scan_topic" value="/scan" /> <!-- Use RPLIDAR scan topic -->
  </include>

  <!-- Move base -->
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml"/>
```


### Mapping the area with LiDAR 
- Terminal 1 | ssh
	- `roslaunch turtlebot_bringup minimal.launch`
- Terminal 2 | ssh
	- `roslaunch rplidar_ros rplidar_a1.launch`
- Terminal 3 | ssh
	- `roslaunch turtlebot_navigation gmapping_demo.launch`
- Terminal 4
	- `rviz`
- After launch all necessary package -> map
- Then save the map


### Example Python script for navigation by using coordinate with speed adjustable
```
#!/usr/bin/env python3
  
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def send_goal(goal_pose):
    # Create an action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # Create a MoveBaseGoal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position and orientation
    goal.target_pose.pose = goal_pose

    # Send the goal and wait for result
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()
  
    # Check the result
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Goal failed or was preempted!")

def increase_speed():
    # Adjust the speed dynamically using cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Allow time for the publisher to connect
    move_cmd = Twist()
    # Set a higher linear velocity
    move_cmd.linear.x = 1.5  # Forward speed (increase this value for faster movement)
    move_cmd.angular.z = 0.0  # No rotation (straight line)
    rospy.loginfo("Increasing speed and moving robot...")
    pub.publish(move_cmd)

def main():
    rospy.init_node('send_goal')
    
    # Define goal 1 position and orientation
    goal1 = PoseStamped()
    goal1.pose.position.x = 0.655
    goal1.pose.position.y = -0.862
    goal1.pose.position.z = 0.000
    goal1.pose.orientation.x = 0.0
    goal1.pose.orientation.y = 0.0
    goal1.pose.orientation.z = -0.0753
    goal1.pose.orientation.w = 0.658

    # Increase speed before sending goal
    increase_speed()

    # Send goal 1
    send_goal(goal1.pose)

    # Define goal 2 position and orientation
    goal2 = PoseStamped()
    goal2.pose.position.x = -18.155
    goal2.pose.position.y = -8.619
    goal2.pose.position.z = 0.000
    goal2.pose.orientation.x = 0.0
    goal2.pose.orientation.y = 0.0
    goal2.pose.orientation.z = 0.754
    goal2.pose.orientation.w = 0.657

    # Send goal 2 after goal 1 is reached
    send_goal(goal2.pose)

    # Define goal 3 position and orientation
    goal3 = PoseStamped()
    goal3.pose.position.x = -19.046
    goal3.pose.position.y = 4.325
    goal3.pose.position.z = 0.000
    goal3.pose.orientation.x = 0.0
    goal3.pose.orientation.y = 0.0
    goal3.pose.orientation.z = -1.0
    goal3.pose.orientation.w = 0.017

    # Send goal 3 after goal 2 is reached
    send_goal(goal3.pose)

    # Define goal 4 position and orientation
    goal4 = PoseStamped()
    goal4.pose.position.x = -10.463
    goal4.pose.position.y = -8.523
    goal4.pose.position.z = 0.000
    goal4.pose.orientation.x = 0.0
    goal4.pose.orientation.y = 0.0
    goal4.pose.orientation.z = -0.752
    goal4.pose.orientation.w = 0.659

    # Send goal 3 after goal 2 is reached
    send_goal(goal4.pose)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

# README

See `Documentation.md` for more information on the different nodes in this project and the protocols. 

## Running the code
1. Clone this repo into the `catkin_ws/src/`.
2. You may need to make sure that some files have executable priveledges on your system. Inside the `rubbish_robot_project` directory run `chmod +x -R .` to make all files executable. 
3. Running the main package can be done using `roslaunch rubbish_robot_project main.launch`. 

### Required Packages
1. Rosbridge is required for the web interface, please install using this command: `sudo apt-get install ros-noetic-rosbridge-suite`. 
2. You may need to install the `explore_lite` to allow for automatic SLAM, this can be done with the command `sudo apt install ros-${ROS_DISTRO}-multirobot-map-merge ros-${ROS_DISTRO}-explore-lite`. 


## Automatic SLAM
1. Make sure you run `catkin_make` in your catkin workspace. 
2. The current implementation of this uses the meeting.world map.
3. Run `roslaunch rubbish_robot_project slam_mapping.launch`, and mapping will commence.
4. The robot *should* stop automatically when it cannot find any more frontiers.
5. When it stops, the map should be saved to `~/maps`
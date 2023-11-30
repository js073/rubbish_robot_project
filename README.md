# README

See `Documentation.md` for more information on the different nodes in this project and the protocols. 

## Running the code
1. Clone this repo into the `catkin_ws/src/`.
2. You may need to make sure that some files have executable priveledges on your system. Inside the `rubbish_robot_project` directory run `chmod +x -R .` to make all files executable. 
3. Running the main package can be done using `roslaunch rubbish_robot_project main.launch`. 

### Required Packages
1. Rosbridge is required for the web interface, please install using this command: `sudo apt-get install ros-noetic-rosbridge-suite`. 
2. You may need to install the `explore_lite` to allow for automatic SLAM, this can be done with the command `sudo apt install ros-${ROS_DISTRO}-multirobot-map-merge ros-${ROS_DISTRO}-explore-lite`. 
3. OpenCV is required for image processing, install using `pip install opencv-python`. 
4. The ROS navigation stack is required, can be installed using `sudo apt-get install ros-noetic-navigation` (assuming ROS Noetic). 

### Simulation
1. Run the Gazebo simulation by running the commanmd `roslaunch rubbish_robot_project simulation.launch`. 

### Main Node
1. With the Gazebo simulation running, run `roslaunch rubbish_robot_project main.launch`. 

### Web Frontend
#### Required Packages
Before running the project, ensure you have the following installed:

- [Node.js](https://nodejs.org/)
- [npm](https://www.npmjs.com/)
- Install VUE CLI `npm install -g @vue/cli`. 

#### Running
1. Clone the repository:
2. `cd my-vue-project`
3. `npm install `
4. run `./node_modules/.bin/eslint --init`
4. `npm run build`
5. `sudo npm run serve`


### Automatic SLAM
1. Make sure you run `catkin_make` in your catkin workspace. 
2. The current implementation of this uses the meeting.world map.
3. Run `roslaunch rubbish_robot_project slam_mapping.launch`, and mapping will commence.
4. The robot *should* stop automatically when it cannot find any more frontiers.
5. When it stops, the map should be saved to `~/maps`



## P3DX
This is the model that we use for our simulation, the code used here is adapted from https://github.com/mario-serna/pioneer_p3dx_model/tree/master. 

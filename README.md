# README

## Automatic SLAM
1. Make sure you run `catkin_make` in your catkin workspace. 
2. The current implementation of this uses the meeting.world map.
3. Run `roslaunch rubbish_robot_project slam_mapping.launch`, and mapping will commence.
4. The robot *should* stop automatically when it cannot find any more frontiers.
5. When it stops, the map should be saved to `~/maps`
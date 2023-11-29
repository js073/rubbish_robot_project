# Documentation

## `main.py`
This node is the main node that will handle running the various other different aspects of the system, and will send/recieve information from the web frontend for controlling the various robot actions. 

### Topics
|Topic|Format|Role|
|-|-|-|
|`/main/commands`|String|The topic that commands sent to the robot will be on|
|`/main/info`|String|The topic that information from the main node will be returned on|

### Commands
|Command|Description|
|-|-|
|`map_start\|<map_name>`|Starts the mapping process, where `<map_name>` is replaced by the name of the map that the user wishes to save. Name is optional and has a default value of `map1`|
|`map_end`|Ends the mapping process and returns the robot to the starting pose|
|`collection_start\|<map_name>`|Starts the collection process, where `<map_name>` is replaced by the name of the map that the user wishes to load for the colleciton process.|
|`collection_end`|Ends the collection process and returns the robot to the starting position|
|`map_list`|Returns a list of the maps in the format `map_list\|<maps>`, where `maps` is a list of maps seperated by commas|
|`robot_state`|Returns the current state of the robot in the form `Robot state: <description>`|
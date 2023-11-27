#!/usr/bin/env python3

import rospy, roslaunch, os, subprocess, rosnode
from std_msgs.msg import String

class mt:
    def __init__(self) -> None:
        self.command_listener = rospy.Subscriber("main/commands", String, self.command_callback, queue_size=100)
        self.info_sender = rospy.Publisher("main/info", String, queue_size=100)
        self.map_send = rospy.Publisher("main/map_cmd", String, queue_size=100)
        self.map_rec = rospy.Subscriber("main/map_info", String, self.map_cmd_callback, queue_size=100)
        self.map_name = "map1"
        self.mapping_nodes = ["/slam_mapping", "/move_base", "/auto_map"]
        self.mapping_launch = None

    def command_callback(self, msg):
        nodes = rosnode.get_node_names()
        mapping_active = all([(n in nodes) for n in self.mapping_nodes])
        print(mapping_active)
        command: str = msg.data
        split_cmd = command.split("|") # Split the incomming command by | - used to denote end of command and start of any variables
        if len(split_cmd) > 0:
            if split_cmd[0] == "map_start":
                self.map_name = split_cmd[1] if len(split_cmd) > 1 else "map1" # The name that the map will be saved under
                self.mapping_start()
            elif split_cmd[0] == "map_end":
                self.mapping_end()
            elif split_cmd[0] == "collection_start":
                self.map_name = split_cmd[1] if len(split_cmd) > 1 else "map1" # Default map name
                self.collection_start()
            elif split_cmd[0] == "collection_end":
                self.collection_end()
            elif split_cmd[0] == "map_list":
                self.get_map_list()
            elif split_cmd[0] == "robot_state":
                self.get_robot_state()

                
    def get_robot_state(self):
        s = "Robot state: "
        if self.collection_launch == None and self.mapping_launch == None:
            s += "Idle"
        elif self.collection_launch != None:
            s += "Robot is currently performing a collection task"
        elif self.mapping_launch != None: 
            s += "Robot is currently performing a mapping task"
        else: 
            s += "An error has occured"
        self.info_sender.publish(s)


    def get_map_list(self):
        s = "map_list|"
        map_dir = os.path.expanduser("~/maps/")
        if os.path.isdir(map_dir):
            yaml_files = []
            folder_files = os.listdir(map_dir)
            for f in folder_files:
                split_filename = f.split(".")
                if len(split_filename) > 1 and split_filename[1] == "yaml":
                    yaml_files.append(split_filename[0])
            s += ','.join(yaml_files)
        self.info_sender.publish(s)
        
            



    def mapping_start(self):
        if self.mapping_launch == None:
            command =['roslaunch', 'rubbish_robot_project', 'slam_mapping.launch', 'map_name:=%s' % self.map_name]
            self.mapping_launch = subprocess.Popen(command, stdout=subprocess.DEVNULL)
            s = "Mapping has started"
            self.info_sender.publish(s)
        else:
            s = "Mapping is already running, if you want to run a new session, then end the current one first"
            self.info_sender.publish(s)

    def mapping_end(self):
        if self.mapping_launch != None:
            s = "Mapping has been complete and saved to file %s, returning to start position" % self.map_name
            self.info_sender.publish(s)
            self.map_send.publish("shutdown")
        else:
            s = "Mapping is not running currently, so cannot be ended"
            self.info_sender.publish(s)

    def collection_start(self):
        if self.mapping_launch != None:
            s = "Mapping is currently running, please end it or wait for it to complete"
            self.info_sender.publish(s)
        else: 
            map_path = os.path.expanduser("~/maps/") + self.map_name + ".yaml"
            command = ['roslaunch', 'rubbish_robot_project', 'robot_task.launch', 'map_path:=%s' % map_path]
            self.collection_launch = subprocess.Popen(command, stdout=subprocess.DEVNULL)
            s = "The collection start has started"
            self.info_sender.publish(s)

    def collection_end(self):
        if self.mapping_launch == None: 
            s = "Nothing to end"
            self.info_sender.publish(s)
        else:
            self.collection_launch.terminate()
            s = "The connection has been terminated"

    def map_cmd_callback(self, msg):
        s = "recieved message %s" % msg.data
        s += "non" if self.mapping_launch == None else "not"
        self.info_sender.publish(s)
        if msg.data == "finished" and self.mapping_launch != None:
            self.mapping_launch.terminate()
            self.mapping_launch = None
            self.info_sender.publish("Robot has returned to starting position and is ready for next actions")
        elif msg.data == "error" and self.mapping_launch != None:
            self.mapping_launch.terminate()
            self.mapping_launch = None
            self.info_sender.publish("An error occured and the robot has not returned to its starting position")            


if __name__ == "__main__":
    rospy.init_node("main_controller")
    mt()
    rospy.spin()
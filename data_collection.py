#!/usr/bin/env python3

import rospy
import matplotlib
from matplotlib import pyplot as plt
from std_msgs.msg import String

class data:
    def __init__(self) -> None:
        self.info_sub = rospy.Subscriber("/info", String, self.string_callback, queue_size=100)
        self.comd_sub = rospy.Subscriber("/main/commands", String, self.command_callback, queue_size=100)
        self.gen_data = []
        self.alter_data = []
        matplotlib.use('agg')

    def string_callback(self, msg):
        string: str = msg.data
        a = string.split(':')
        if "Map" in string:
            self.alter_data.append(float(a[1]))
        elif "Goal" in string:
            self.gen_data.append(float(a[1]))
            rospy.loginfo(self.gen_data[0])

    def command_callback(self, msg):
        string: str = msg.data
        if "gen" in string:
            rospy.loginfo(len(self.gen_data))
            plt.boxplot(self.gen_data)
            plt.boxplot(self.alter_data)
            plt.savefig("test.png")

if __name__ == "__main__":
    rospy.init_node("collection")
    data()
    rospy.spin()
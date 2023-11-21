#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class simple_command_sender:
    def __init__(self) -> None:
        self.command_sender = rospy.Publisher("main/commands", String, queue_size=100)
        # self.info_reciever = rospy.Subscriber("main/info", String, self.message_callback, queue_size=100)

    def message_callback(self, msg):
        s = "Recieved message | %s" % msg.data
        print(s)

    def send_commands(self):
        user_input = input("Enter command")
        if user_input == "exit":
            return False
        else:
            self.command_sender.publish(user_input)
            return True

if __name__ == "__main__":
    rospy.init_node("command_send")
    a = simple_command_sender()
    b = True
    while b:
        b = a.send_commands()
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class ObstacleMover:
    def __init__(self):
        rospy.init_node('obstacle_mover')
        # Change the topic to match your robot's velocity command topic
        self.pub = rospy.Publisher('/dynamic_sphere/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.distance = 3  # meters
        self.speed = 0.6  # meters per second

    def move_straight(self, forward):
        vel_cmd = Twist()
        # Positive x is forward, negative x is backward
        vel_cmd.linear.x = self.speed if forward else -self.speed
        distance_moved = 0.0

        while distance_moved < self.distance:
            self.pub.publish(vel_cmd)
            self.rate.sleep()
            distance_moved += self.speed / 10.0

        # Stop the robot
        vel_cmd.linear.x = 0
        self.pub.publish(vel_cmd)

    def turn(self, clockwise):
        vel_cmd = Twist()
        # Positive z is left turn, negative z is right turn
        vel_cmd.angular.z = -1.57 if clockwise else 1.57  # 90 degrees in radians
        self.pub.publish(vel_cmd)
        # Adjust sleep time if necessary for a 90-degree turn
        rospy.sleep(1)

        # Stop the turn
        vel_cmd.angular.z = 0
        self.pub.publish(vel_cmd)

    def run(self):
        while not rospy.is_shutdown():
            # Move straight then turn left
            self.move_straight(True)
            self.turn(False)  # False for counter-clockwise turn (left)
            self.move_straight(True)

            # Move backward then turn right
            self.move_straight(False)
            self.turn(True)  # True for clockwise turn (right)
            self.move_straight(False)

if __name__ == '__main__':
    mover = ObstacleMover()
    mover.run()

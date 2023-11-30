#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class ObstacleMover:
    def __init__(self):
        rospy.init_node('obstacle_mover')
        self.pub_a = rospy.Publisher('/moving_box_1/cmd_vel', Twist, queue_size=10)
        self.pub_b = rospy.Publisher('/obstacle_box_2/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.distance = 30  # meters
        self.speed = 0.5  # meters per second

    def move_obstacle(self, pub, forward):
        vel_cmd = Twist()
        vel_cmd.linear.x = self.speed if forward else -self.speed
        distance_moved = 0.0

        while distance_moved < self.distance:
            pub.publish(vel_cmd)
            self.rate.sleep()
            distance_moved += self.speed / 10.0

        # Stopping the obstacle
        vel_cmd.linear.x = 0
        pub.publish(vel_cmd)

    def run(self):
        while not rospy.is_shutdown():
            # Move forward and back for both obstacles
            self.move_obstacle(self.pub_a, True)
            self.move_obstacle(self.pub_b, True)
            self.move_obstacle(self.pub_a, False)
            self.move_obstacle(self.pub_b, False)

if __name__ == '__main__':
    mover = ObstacleMover()
    mover.run()

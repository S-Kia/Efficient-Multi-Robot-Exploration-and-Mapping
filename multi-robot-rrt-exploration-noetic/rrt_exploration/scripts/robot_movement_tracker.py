#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class RobotMovementTracker:
    def __init__(self):
        self.robot_status = {"tb3_0": False, "tb3_1": False, "tb3_2": False}  # Movement status for each robot
        self.start_time = None
        self.all_stopped = True  # Tracks if all robots are stopped

        # Subscribers for each robot's velocity topic
        rospy.Subscriber("/tb3_0/cmd_vel", Twist, self.callback, "tb3_0")
        rospy.Subscriber("/tb3_1/cmd_vel", Twist, self.callback, "tb3_1")
        rospy.Subscriber("/tb3_2/cmd_vel", Twist, self.callback, "tb3_2")

    def callback(self, msg, robot_name):
        # Determine if the robot is moving
        is_moving = not (msg.linear.x == 0.0 and msg.angular.z == 0.0)

        # Update status
        self.robot_status[robot_name] = is_moving

        if is_moving:
            if self.all_stopped:  # If all robots were previously stopped, start the timer
                self.start_time = time.time()
                self.all_stopped = False
                rospy.loginfo("Robots started moving.")
        else:
            # Check if all robots are stopped
            if all(not status for status in self.robot_status.values()):
                if not self.all_stopped:  # If previously moving, stop the timer
                    elapsed_time = time.time() - self.start_time
                    rospy.loginfo(f"All robots stopped. Total movement time: {elapsed_time:.2f} seconds.")
                    self.all_stopped = True

if __name__ == "__main__":
    rospy.init_node("robot_movement_tracker")
    tracker = RobotMovementTracker()
    rospy.spin()


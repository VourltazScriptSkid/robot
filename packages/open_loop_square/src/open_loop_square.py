#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped
import math

class DriveSquare:
    def __init__(self):
        rospy.init_node('drive_square_node', anonymous=True)

        # Publisher and Subscribers
        self.pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/stripe/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/stripe/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/stripe/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

        self.cmd_msg = Twist2DStamped()
        self.left_ticks = 0
        self.right_ticks = 0

        # Constants
        self.TICKS_PER_METER = 723
        self.TICKS_PER_90_DEG = 90  # experimentally determined (adjust if needed)

        # Motion control state
        self.state = "idle"
        self.start_left_ticks = 0
        self.start_right_ticks = 0
        self.edge_index = 0  # 0–3 for each square side

        # Start timer loop
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)  # 100 Hz

        rospy.loginfo("Closed-loop drive_square_node initialized")
        rospy.spin()

    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING" and self.state == "idle":
            self.state = "moving_straight"
            self.start_left_ticks = self.left_ticks
            rospy.loginfo("Starting closed-loop square movement")

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def timer_callback(self, event):
        if self.state == "moving_straight":
            ticks_traveled = abs(self.left_ticks - self.start_left_ticks)
            if ticks_traveled < self.TICKS_PER_METER:
                self.cmd_msg.v = 0.2
                self.cmd_msg.omega = 0.0
            else:
                self.state = "turning"
                self.start_left_ticks = self.left_ticks
                self.start_right_ticks = self.right_ticks
                rospy.loginfo(f"Completed side {self.edge_index + 1}, now turning")
                self.cmd_msg.v = 0.0
                self.cmd_msg.omega = 0.0

        elif self.state == "turning":
            delta_ticks = abs((self.right_ticks - self.start_right_ticks) - (self.left_ticks - self.start_left_ticks))
            if delta_ticks < self.TICKS_PER_90_DEG:
                self.cmd_msg.v = 0.0
                self.cmd_msg.omega = 1.0
            else:
                self.edge_index += 1
                if self.edge_index < 4:
                    self.state = "moving_straight"
                    self.start_left_ticks = self.left_ticks
                    rospy.loginfo(f"Turn complete. Starting side {self.edge_index + 1}")
                else:
                    self.state = "done"
                    rospy.loginfo("Square complete")
                self.cmd_msg.v = 0.0
                self.cmd_msg.omega = 0.0

        elif self.state == "done":
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 0.0

        else:
            # Idle or unknown state
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 0.0

        # Publish command
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.cmd_msg)

if __name__ == '__main__':
    try:
        DriveSquare()
    except rospy.ROSInterruptException:
        pass

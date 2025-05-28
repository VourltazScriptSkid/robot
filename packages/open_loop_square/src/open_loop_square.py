#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped
import math

class Drive_Square:
    def __init__(self):
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Choose the test mode here
        self.test_mode = "square_floor_1"  # Change to: "straight_0.2", "rotate_1.0", etc.

        self.cmd_msg = Twist2DStamped()
        self.left_ticks = 0
        self.right_ticks = 0

        self.TICKS_PER_METER = 738
        self.TICKS_PER_90_DEG = 90

        self.pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/stripe/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/stripe/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/stripe/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.run_demo()

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_straight(self, distance, speed):
        ticks_needed = abs(distance * self.TICKS_PER_METER)
        initial_ticks = self.left_ticks
        rate = rospy.Rate(60)
        direction = 1 if distance > 0 else -1

        while abs(self.left_ticks - initial_ticks) < ticks_needed:
            self.cmd_msg.v = speed * direction
            self.cmd_msg.omega = 0.0
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()

    def rotate_in_place(self, angle_deg, speed):
        ticks_needed = abs((angle_deg / 90.0) * self.TICKS_PER_90_DEG)
        initial_left = self.left_ticks
        initial_right = self.right_ticks
        rate = rospy.Rate(60)
        direction = 1 if angle_deg > 0 else -1

        while abs((self.right_ticks - initial_right) - (self.left_ticks - initial_left)) < ticks_needed:
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = speed * direction
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()

    def draw_square(self, speed_linear, speed_angular):
        for _ in range(4):
            self.move_straight(1.0, speed_linear)
            self.rotate_in_place(90, speed_angular)

    def run_demo(self):
        if self.test_mode == "straight_0.2":
            self.move_straight(1.0, 0.2)
        elif self.test_mode == "straight_0.4":
            self.move_straight(1.0, 0.4)
        elif self.test_mode == "rotate_1.0":
            self.rotate_in_place(90, 1.0)
        elif self.test_mode == "rotate_2.0":
            self.rotate_in_place(90, 2.0)
        elif self.test_mode == "square_floor_1":
            self.draw_square(0.2, 1.0)
        elif self.test_mode == "square_floor_2":
            self.draw_square(0.2, 1.0)
        else:
            rospy.logwarn("Invalid test mode selected.")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = Drive_Square()
        controller.run()
    except rospy.ROSInterruptException:
        pass

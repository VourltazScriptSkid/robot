#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class DriveSquare:
    def __init__(self):
        rospy.init_node('drive_square_node', anonymous=True)

        self.test_mode = "square_floor_1"

        self.cmd_msg = Twist2DStamped()
        self.left_ticks = 0
        self.right_ticks = 0
        self.tof_distance = float('inf')  # default: no obstacle

        self.TICKS_PER_METER = 738
        self.TICKS_PER_90_DEG = 90
        self.STOP_DISTANCE = 0.2  # meters

        self.pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/stripe/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/stripe/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/stripe/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/stripe/front_center_tof_driver_node/range', Range, self.tof_callback)

        self.received_encoder_data = False

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING" and not self.received_encoder_data:
            self.received_encoder_data = True
            rospy.sleep(1.0)
            self.run_demo()

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def tof_callback(self, msg):
        self.tof_distance = msg.range

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_straight(self, distance_m, speed_mps):
        direction = 1 if distance_m > 0 else -1
        speed_mps = abs(speed_mps) * direction

        initial_left = self.left_ticks
        initial_right = self.right_ticks
        ticks_needed = abs(distance_m) * self.TICKS_PER_METER
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            delta_left = self.left_ticks - initial_left
            delta_right = self.right_ticks - initial_right
            avg_ticks = (delta_left + delta_right) / 2.0

            if abs(avg_ticks) >= ticks_needed:
                break

            if self.tof_distance < self.STOP_DISTANCE:
                rospy.loginfo("Obstacle detected. Pausing...")
                self.stop_robot()
                while self.tof_distance < self.STOP_DISTANCE and not rospy.is_shutdown():
                    rate.sleep()
                rospy.loginfo("Obstacle cleared. Resuming...")

            self.cmd_msg.v = speed_mps
            self.cmd_msg.omega = 0.0
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)

    def rotate_in_place(self, angle_deg, angular_speed):
        direction = 1 if angle_deg > 0 else -1
        angular_speed = abs(angular_speed) * direction

        initial_left = self.left_ticks
        initial_right = self.right_ticks
        ticks_needed = abs(angle_deg) / 90.0 * self.TICKS_PER_90_DEG
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            delta_left = self.left_ticks - initial_left
            delta_right = self.right_ticks - initial_right
            diff_ticks = abs(delta_right - delta_left)

            if diff_ticks >= ticks_needed:
                break

            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)

    def draw_square(self, speed_linear, speed_angular):
        for _ in range(4):
            self.move_straight(1.0, speed_linear)
            self.rotate_in_place(90, speed_angular)

    def run_demo(self):
        mode = self.test_mode
        if mode == "straight_0.2":
            self.move_straight(1.0, 0.2)
        elif mode == "straight_0.4":
            self.move_straight(1.0, 0.4)
        elif mode == "rotate_1.0":
            self.rotate_in_place(90, 1.0)
        elif mode == "rotate_2.0":
            self.rotate_in_place(90, 2.0)
        elif mode == "square_floor_1":
            self.draw_square(0.2, 1.0)
        elif mode == "square_floor_2":
            self.draw_square(0.2, 1.0)
        else:
            rospy.logwarn("Invalid test mode selected.")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DriveSquare()
        node.run()
    except rospy.ROSInterruptException:
        pass

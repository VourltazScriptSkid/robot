#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
import math
 
class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        self.left_ticks = 0
        self.right_ticks = 0

        
        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/stripe/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/stripe/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/stripe/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

        
    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.move_straight(1)
            self.rotate_in_place(90)
            self.move_straight(1)
            self.rotate_in_place(90)
            self.move_straight(1)
            self.rotate_in_place(90)
            self.move_straight(1)
            self.rotate_in_place(90)
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
    	rospy.spin() # keeps node from exiting until node has shutdown

    # Robot drives in a square and then stops
    def move_robot(self):

        #YOUR CODE GOES HERE#
        rate = rospy.Rate(10)  # 10 Hz control rate
        linear_speed = 0.2      # meters per second
        turn_speed = 4.0        # radians per second

        forward_duration = 2.5  # time to travel 1 meter
        turn_duration = math.pi / 2 / turn_speed * 0.5 # time to turn 90 degrees

        for i in range(4):
            rospy.loginfo(f"Side {i+1}: Moving forward")
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < forward_duration:
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = linear_speed
                self.cmd_msg.omega = 0.0
                self.pub.publish(self.cmd_msg)
                rate.sleep()

            rospy.loginfo(f"Side {i+1}: Turning 90 degrees")
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < turn_duration:
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.0
                self.cmd_msg.omega = turn_speed
                self.pub.publish(self.cmd_msg)
                rate.sleep()

            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.sleep(0.1)

    def move_straight(self, distance):
        TICKS_PER_METER = 723
        ticks_needed = distance * TICKS_PER_METER
        self.initial_left_ticks = self.left_ticks

        rate = rospy.Rate(10)
        while abs(self.left_ticks - self.initial_left_ticks) < ticks_needed:
            self.cmd_msg.v = 0.2
            self.cmd_msg.omega = 0.0
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()


    def rotate_in_place(self, angle_deg):
        TICKS_PER_FULL_ROTATION = 359
        ticks_needed = (angle_deg / 360.0) * TICKS_PER_FULL_ROTATION
        self.initial_left_ticks = self.left_ticks
        self.initial_right_ticks = self.right_ticks

        rate = rospy.Rate(10)
        while abs((self.right_ticks - self.initial_right_ticks) - 
                (self.left_ticks - self.initial_left_ticks)) < ticks_needed:
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 1.0
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data  # adjust if it's not .data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data



        ######################
                
        self.stop_robot()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

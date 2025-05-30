#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Float32

class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_tof = False  # Flag to ignore tof detections temporarily

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/stripe/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/stripe/tof_front_center', Float32, self.tof_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # ToF Sensor Callback
    def tof_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

        # Ignore ToF readings if cooldown is active
        if self.ignore_tof:
            return

        self.move_robot(msg.data)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def tof_cooldown_done(self, event):
        rospy.loginfo("ToF cooldown complete, resuming obstacle detection.")
        self.ignore_tof = False

    def move_robot(self, distance):

        #### YOUR CODE GOES HERE ####

        if distance > 0.3:  # No object nearby
            return

        # Stop lane following temporarily
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Stop the robot
        self.stop_robot()
        rospy.loginfo("Obstacle detected. Waiting up to 5 seconds...")

        # Wait up to 5 seconds to see if it moves
        wait_time = rospy.Time.now()
        timeout = rospy.Duration(5.0)

        rate = rospy.Rate(10)
        while rospy.Time.now() - wait_time < timeout:
            distance_msg = rospy.wait_for_message('/stripe/tof_front_center', Float32, timeout=0.5)
            if distance_msg.data > 0.3:
                rospy.loginfo("Obstacle cleared. Resuming lane following.")
                self.set_state("LANE_FOLLOWING")
                return
            rate.sleep()

        # Overtake maneuver (open loop)
        rospy.loginfo("Obstacle still there. Executing overtake maneuver.")
        
        # Step 1: Curve left
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.2
        cmd_msg.omega = 2.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(1.0)

        # Step 2: Go straight
        cmd_msg.v = 0.3
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(1.2)

        # Step 3: Curve right
        cmd_msg.v = 0.2
        cmd_msg.omega = -2.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(1.0)

        # Step 4: Straighten out
        cmd_msg.v = 0.3
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(1.0)

        # Resume lane following
        self.set_state("LANE_FOLLOWING")

        # Ignore ToF detection briefly so we don't re-trigger instantly
        self.ignore_tof = True
        rospy.Timer(rospy.Duration(3.0), self.tof_cooldown_done, oneshot=True)

        #############################

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass

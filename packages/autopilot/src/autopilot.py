#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_tags = False  # NEW: Flag to ignore AprilTags temporarily

        rospy.on_shutdown(self.clean_shutdown)

        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/stripe/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/stripe/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin()

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

        # Ignore AprilTags if cooldown is active
        if self.ignore_tags:
            return
        
        self.move_robot(msg.detections)

    def tag_cooldown_done(self, event):
        rospy.loginfo("AprilTag cooldown complete, resuming tag detection.")
        self.ignore_tags = False

    def move_robot(self, detections):

        #### YOUR CODE GOES HERE ####

        if len(detections) == 0:
            return

        # Stop lane following temporarily
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Stop the robot
        self.stop_robot()
        rospy.loginfo("Stop sign detected. Stopping robot for 3 seconds.")
        rospy.sleep(3.0)

        # Resume lane following
        self.set_state("LANE_FOLLOWING")

        # Ignore AprilTags for 5 seconds while driving away
        self.ignore_tags = True
        rospy.Timer(rospy.Duration(5.0), self.tag_cooldown_done, oneshot=True)

        #############################


if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
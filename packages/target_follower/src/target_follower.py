#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/stripe/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
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

    def move_robot(self, detections):
        if len(detections) == 0:
            # No tag detected → rotate slowly to search
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.02  # Try a small value like 0.3–0.7
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.loginfo("No tag detected. Searching by rotating...")
            rospy.sleep(1)
            return

        # Tag detected → align with it
        x = detections[0].transform.translation.x
        z = detections[0].transform.translation.z

        rospy.loginfo("x, z: %f %f", x, z)

        # --- Control parameters ---
        Kp = 2.5
        max_omega = 3.0
        min_omega = 0.2
        deadzone = 0.03

        # --- Calculate control ---
        error = x
        if abs(error) < deadzone:
            omega = 0.0
        else:
            omega = Kp * error
            if omega > 0:
                omega = max(min_omega, min(omega, max_omega))
            else:
                omega = min(-min_omega, max(omega, -max_omega))

        # --- Publish command ---
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)



if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
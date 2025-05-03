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
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if len(detections) == 0:
            # No tag detected → slowly rotate to search
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.2  # Just enough to overcome friction
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.loginfo("No tag detected. Rotating slowly to search...")
            return

        # --- Get AprilTag x position from detection ---
        x = detections[0].transform.translation.x  # Left/right offset in meters
        rospy.loginfo("Tag detected: x = %.3f", x)

        # --- Control Parameters ---
        Kp = 0.4           # Proportional gain (tune as needed)
        max_omega = 0.6    # Max turn rate
        min_omega = 0.2    # Min turn rate to overcome friction
        deadzone = 0.03    # Do nothing if error is within this

        # --- Control Logic ---
        if abs(x) < deadzone:
            omega = 0.0
            rospy.loginfo("Tag centered. Holding position.")
        else:
            raw_omega = Kp * x  # P controller

            # Apply minimum to overcome friction
            if abs(raw_omega) < min_omega:
                omega = min_omega if raw_omega > 0 else -min_omega
            else:
                omega = raw_omega

            # Clamp to maximum
            omega = max(-max_omega, min(omega, max_omega))
            rospy.loginfo("Tracking tag. Error = %.3f | Omega = %.3f", x, omega)

        # Send command
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

        # Optional: Delay to slow down command rate (can help stabilize)
        rospy.sleep(50)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
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
            rospy.loginfo("No tag detected. Staying stationary.")
            self.stop_robot()
            return

        # --- Tag detected ---
        x = detections[0].transform.translation.x  # left/right offset
        z = detections[0].transform.translation.z  # forward distance to tag
        rospy.loginfo("Tag detected: x = %.3f, z = %.3f", x, z)

        # Control parameters
        Kp_angle = 0.4         # Gain for turning
        Kp_dist = 0.8          # Gain for forward motion

        max_omega = 0.6        # Maximum angular speed
        min_omega = 0.2        # Minimum angular speed to overcome inertia
        deadzone_x = 0.05      # Tag center tolerance

        max_v = 0.2            # Maximum forward speed
        min_v = 0.05           # Minimum linear speed to overcome inertia
        deadzone_z = 0.02      # Distance deadzone (to hold position)

        target_distance = 0.2  # Desired distance to maintain from the tag

        # Compute error
        lateral_error = -x
        distance_error = z - target_distance

        # --- Angular control (turning to center tag) ---
        if abs(lateral_error) < deadzone_x:
            omega = 0.0
            rospy.loginfo("Tag centered. No turn.")
        else:
            raw_omega = Kp_angle * lateral_error
            if abs(raw_omega) < min_omega:
                omega = min_omega if raw_omega > 0 else -min_omega
            else:
                omega = raw_omega
            omega = max(-max_omega, min(omega, max_omega))

        # --- Linear control (forward/backward) ---
        if abs(distance_error) < deadzone_z:
            v = 0.0
            rospy.loginfo("At desired distance. Holding position.")
        else:
            raw_v = Kp_dist * distance_error
            if abs(raw_v) < min_v:
                v = min_v if raw_v > 0 else -min_v
            else:
                v = raw_v
            v = max(-max_v, min(v, max_v))

        # Send command
        cmd_msg.v = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.loginfo("Tracking. Distance error: %.3f, Lateral error: %.3f, v: %.3f, omega: %.3f", distance_error, lateral_error, v, omega)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass

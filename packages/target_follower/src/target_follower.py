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
            # Step-wise search: rotate briefly, then pause
            rospy.loginfo("No tag detected. Rotating briefly to search...")

            # Step 1: Rotate
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.3
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.2)  # Rotate for 0.2 seconds

            # Step 2: Stop and check again
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.3)  # Pause to let camera process

            return

        # --- Tag detected ---
        x = detections[0].transform.translation.x
        rospy.loginfo("Tag detected: x = %.3f", x)

        # Control parameters
        Kp = 0.3          # Proportional gain
        max_omega = 0.5   # Maximum turning speed
        min_omega = 0.2   # Minimum to overcome friction
        deadzone = 0.1   # Smaller deadzone to stay responsive

        # Compute control
        error = -x

        if abs(error) < deadzone:
            omega = 0.0
            rospy.loginfo("Tag centered. Holding position.")
        else:
            if abs(error) < 2 * deadzone:
                # Slow correction in near-center range
                Kp_local = 0.15
            else:
                Kp_local = Kp

            raw_omega = Kp_local * error
            if abs(raw_omega) < min_omega:
                omega = min_omega if raw_omega > 0 else -min_omega
            else:
                omega = raw_omega
            omega = max(-max_omega, min(omega, max_omega))


        # Send command
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.loginfo("Tracking. Error: %.3f, Omega: %.3f", error, omega)

        


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
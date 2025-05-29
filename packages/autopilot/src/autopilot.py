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
        self.tof_distance = None
        self.blocked_start_time = None
        self.waiting = False
        self.overtaking = False

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "stripe" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/stripe/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/stripe/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/stripe/tof_driver_node/range', Float32, self.tof_callback, queue_size=1)
        ################################################################

        # Start driving timer
        rospy.Timer(rospy.Duration(0.1), self.drive_loop)

        rospy.spin() # Spin forever but listen to message callbacks

    # ToF Sensor Callback
    def tof_callback(self, msg):
        self.tof_distance = msg.data

        if self.robot_state != "LANE_FOLLOWING":
            return

        if self.tof_distance < 0.1:
            if not self.waiting:
                # First detection - stop and start waiting
                rospy.loginfo("Obstacle detected. Stopping and waiting...")
                self.set_state("NORMAL_JOYSTICK_CONTROL")
                self.stop_robot()
                self.blocked_start_time = rospy.Time.now()
                self.waiting = True
            else:
                # Still blocked - check if 5 seconds passed
                if (rospy.Time.now() - self.blocked_start_time).to_sec() >= 5.0:
                    rospy.loginfo("Obstacle did not move. Initiating overtake.")
                    self.overtake()
        else:
            if self.waiting:
                rospy.loginfo("Obstacle cleared. Resuming lane following.")
                self.set_state("LANE_FOLLOWING")
                self.waiting = False
                self.blocked_start_time = None

    # Drives forward if no obstacle is detected
    def drive_loop(self, event):
        if self.robot_state == "LANE_FOLLOWING" and not self.overtaking:
            self.send_cmd(0.2, 0.0)  # Forward at slow speed

    # Stop Robot before node has shut down. This ensures the robot doesn't keep moving
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        self.send_cmd(0.0, 0.0)

    # Helper to send a velocity command
    def send_cmd(self, v, omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

    # Set robot FSM state
    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    # Perform a simple overtaking maneuver
    def overtake(self):
        self.overtaking = True

        # Step 1: Small left turn
        self.send_cmd(0.1, 2.0)
        rospy.sleep(1.0)

        # Step 2: Drive forward past obstacle
        self.send_cmd(0.3, 0.0)
        rospy.sleep(1.5)

        # Step 3: Small right turn to return to lane
        self.send_cmd(0.1, -2.0)
        rospy.sleep(1.0)

        # Step 4: Stop briefly and return to lane following
        self.stop_robot()
        self.set_state("LANE_FOLLOWING")

        # Reset flags
        self.waiting = False
        self.overtaking = False
        self.blocked_start_time = None
        rospy.loginfo("Overtake complete. Resumed lane following.")

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass

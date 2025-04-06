#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
from math import atan2, pi
import time 

class TurtlesimStraightsAndTurns:
    def __init__(self):
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        self.goal_angle = 0
        self.angle_goal_active = False
        self.current_theta = 0
        self.initial_theta = 0
        self.rotate_ccw = True

        self.pose = Pose()

        self.position_goal = Point()
        self.position_goal_active = False
        self.state = "rotate_to_goal"

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.loginfo("Initialized node!")
        rospy.spin()

    def pose_callback(self, msg):
        self.current_theta = msg.theta
        self.pose = msg

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        self.angle_goal_active = True
        self.initial_theta = self.current_theta
        self.rotate_ccw = True if msg.data >= 0 else False

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.dist_goal_active = True
        self.forward_movement = True if msg.data >= 0 else False

    def goal_position_callback(self, msg):
        self.position_goal = msg
        self.position_goal_active = True
        self.state = "rotate_to_goal"

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def timer_callback(self, msg):
        cmd = Twist()

        if self.angle_goal_active:
            angle_travelled = abs(self.current_theta - self.initial_theta)
            if angle_travelled >= abs(self.goal_angle) - 0.01:
                self.velocity_publisher.publish(Twist())
                self.angle_goal_active = False
            else:
                cmd.angular.z = 1.0 if self.rotate_ccw else -1.0
                self.velocity_publisher.publish(cmd)

        elif self.dist_goal_active:
            distance_remaining = abs(self.goal_distance) - abs(self.last_distance)
            if distance_remaining <= 0.05:
                self.velocity_publisher.publish(Twist())
                self.dist_goal_active = False
            else:
                cmd.linear.x = 1.0 if self.forward_movement else -1.0
                self.velocity_publisher.publish(cmd)

        elif self.position_goal_active:
            dx = self.position_goal.x - self.pose.x
            dy = self.position_goal.y - self.pose.y
            goal_angle = atan2(dy, dx)
            distance_to_goal = (dx**2 + dy**2)**0.5

            if self.state == "rotate_to_goal":
                angle_diff = self.normalize_angle(goal_angle - self.pose.theta)
                if abs(angle_diff) < 0.05:
                    self.state = "move_to_goal"
                else:
                    cmd.angular.z = 1.5 if angle_diff > 0 else -1.5
                    self.velocity_publisher.publish(cmd)

            elif self.state == "move_to_goal":
                if distance_to_goal < 0.1:
                    self.position_goal_active = False
                    self.state = "rotate_to_goal"
                    self.velocity_publisher.publish(Twist())
                else:
                    cmd.linear.x = 1.0
                    self.velocity_publisher.publish(cmd)

if __name__ == '__main__': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass

#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64,self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64,self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64,self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self,msg):
        pass

    def distance_callback(self,msg):
        self.last_distance = msg.data

    def goal_angle_callback(self,msg):
        pass

    def goal_distance_callback(self,msg):
        ########## YOUR CODE GOES HERE ##########
        # Set goal_distance, dist_goal_active and forward_movement variables here
        self.goal_distance = msg.data
        self.dist_goal_active = True
        self.forward_movement = True if msg.data >= 0 else False

        ###########################################

    def timer_callback(self,msg):
        ########## YOUR CODE GOES HERE ##########
        # If a goal is active, first check if the goal is reached (it's OK if the goal is not perfectly reached)
        # Then publish a cmd_vel message

        if self.dist_goal_active:
            distance_remaining = abs(self.goal_distance) - abs(self.last_distance)

            if distance_remaining <= 0.05:
                # Stop turtle
                cmd = Twist()
                self.velocity_publisher.publish(cmd)
                self.dist_goal_active = False
            else:
                cmd = Twist()
                cmd.linear.x = 1.0 if self.forward_movement else -1.0
                self.velocity_publisher.publish(cmd)

        ###########################################

if __name__ == '__main__': 

    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
        

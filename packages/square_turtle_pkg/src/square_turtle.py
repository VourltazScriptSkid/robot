#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist 
import time 

def move_turtle_square(): 
    rospy.init_node('turtlesim_square_node', anonymous=True)
    
    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    rospy.loginfo("Turtles are great at drawing squares!")

    ########## YOUR CODE GOES HERE ##########
        while not rospy.is_shutdown():
        rate = rospy.Rate(1)  # 1 Hz, adjust as needed

        # Move forward
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 2.0
        cmd_vel_msg.angular.z = 0.0
        velocity_publisher.publish(cmd_vel_msg)
        rate.sleep()

        # Stop
        cmd_vel_msg = Twist()
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(0.5)

        # Turn 90 degrees
        cmd_vel_msg.angular.z = 1.57  # rad/s (~90 degrees/sec)
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(1.0)

        # Stop
        cmd_vel_msg = Twist()
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(0.5)

        ###########################################

if __name__ == '__main__': 

    try: 
        move_turtle_square() 
    except rospy.ROSInterruptException: 
        pass
        

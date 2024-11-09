#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_in_rectangle():
    # Initialize the ROS node
    rospy.init_node('move_turtle_rectangle_node', anonymous=True)
    
    # Create a publisher to send velocity commands to the turtle
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Define the rate at which the loop will run (10 Hz)
    rate = rospy.Rate(10)
    
    # Create a Twist message to set linear and angular velocity
    vel_msg = Twist()

    # Set linear velocity for forward motion (adjust speed for the rectangle size)
    vel_msg.linear.x = 1.0  # Move forward with a speed of 1.0 m/s
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    
    # Set angular velocity for turning 90 degrees (adjust turn speed as needed)
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 1.0  # Turning speed for 90-degree turns (adjust as needed)

    rospy.loginfo("Moving the turtle in a continuous rectangular path...")
    
    # Infinite loop to repeat the rectangular movement
    while not rospy.is_shutdown():
        for _ in range(2):  # We will repeat the movement twice to create a rectangle (two long sides)
            # Move forward for the long side of the rectangle
            vel_msg.linear.x = 1.0  # Set forward velocity
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 3:  # Move for 3 seconds (adjust for rectangle side length)
                velocity_publisher.publish(vel_msg)
                rate.sleep()
            
            # Stop the turtle after moving forward
            vel_msg.linear.x = 0.0
            velocity_publisher.publish(vel_msg)
            time.sleep(1)  # Pause for 1 second before turning
            
            # Turn 90 degrees (make sure the turn completes a sharp 90-degree angle)
            vel_msg.angular.z = 1.0  # Set turning velocity
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 1.5:  # Turn for 1.5 seconds (adjust for 90-degree turn)
                velocity_publisher.publish(vel_msg)
                rate.sleep()
            
            # Stop turning
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            time.sleep(1)  # Pause for 1 second before moving forward again

        rospy.loginfo("Completed one rectangular cycle. Starting another...")

if __name__ == '__main__':
    try:
        # Call the function to move the turtle in a rectangular path
        move_turtle_in_rectangle()
    except rospy.ROSInterruptException:
        pass


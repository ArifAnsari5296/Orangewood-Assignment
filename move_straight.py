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

    # Set the speed for forward motion
    vel_msg.linear.x = 1.0  # Speed of 1.0 m/s for forward motion
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    
    # Set the turning speed (angular velocity)
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 1.0  # Turning speed for 90-degree turns (adjust this value if needed)

    rospy.loginfo("Moving the turtle in a continuous rectangular path...")
    
    # Infinite loop to repeat the rectangular movement
    while not rospy.is_shutdown():
        for _ in range(2):  # Repeat the following twice to create a rectangle (two long sides)
            # Move forward for the first side (long side of the rectangle)
            vel_msg.linear.x = 2.0  # Set speed for moving forward
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 3:  # Move for 3 seconds (adjust for rectangle size)
                velocity_publisher.publish(vel_msg)
                rate.sleep()
            
            # Stop the turtle after moving forward
            vel_msg.linear.x = 0.0
            velocity_publisher.publish(vel_msg)
            time.sleep(1)  # Pause for 1 second before turning
            
            # Turn 90 degrees (right turn)
            vel_msg.angular.z = 1.0  # Turning speed for 90-degree turn
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 1.5:  # Turn for 1.5 seconds (adjust for 90 degrees)
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


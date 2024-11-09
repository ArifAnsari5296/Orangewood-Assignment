#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_in_rectangle():
    rospy.init_node('move_turtle_rectangle_node', anonymous=True)
    
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)
    
    vel_msg = Twist()

    vel_msg.linear.x = 1.0  
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 1.0 

    rospy.loginfo("Moving the turtle in a continuous rectangular path...")
    
    while not rospy.is_shutdown():
        for _ in range(2):  
            vel_msg.linear.x = 1.0  
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 3:  
                velocity_publisher.publish(vel_msg)
                rate.sleep()
            
            vel_msg.linear.x = 0.0
            velocity_publisher.publish(vel_msg)
            time.sleep(1)  
            
            vel_msg.angular.z = 1.0  
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 1.5: 
                velocity_publisher.publish(vel_msg)
                rate.sleep()
            
            # Stop turning
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            time.sleep(1)  

        rospy.loginfo("Completed one rectangular cycle. Starting another...")

if __name__ == '__main__':
    try:
        move_turtle_in_rectangle()
    except rospy.ROSInterruptException:
        pass


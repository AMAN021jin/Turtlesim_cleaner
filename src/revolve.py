#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# import get_time
def move():
    rospy.init_node('turtle_revolve', anonymous=True)
    vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size =10)
    vel_msg = Twist()    #define a twist object bcoz twist message which is of geometry_msgs type,, contains information about speed of turtle or robot 

  
    radius = int(input("Enter Radius"))
    speed = int(input("Enter speed"))
    distance_1 = 2*3.14*radius
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = speed/radius

    i = 0
    t0 = rospy.get_rostime().to_sec()
    # print(t0)
    while not rospy.is_shutdown():
        cur_dis = 0

        while (cur_dis<distance_1):

            vel.publish(vel_msg)
        
            t1 = rospy.get_rostime().to_sec()
            cur_dis = speed*(t1-t0)
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        vel.publish(vel_msg)    

        print(t0)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass    
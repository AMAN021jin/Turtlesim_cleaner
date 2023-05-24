#!/usr/bin/env python3

from math import atan2, sqrt, pow
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import matplotlib.pyplot as plt
kp = 0.7
kd = 1.1
ki = 0.01

class turtlebot():
    
    def __init__(self):
        rospy.init_node('Go to goal', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.posecallback)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def posecallback(self, msg):
        self.pose = msg
    
        self.pose.x = self.pose.x
        self.pose.y = self.pose.y
        self.pose.theta = self.pose.theta

    def moveGoal(self):

        vel_msgs = Twist()

        gp = Pose()                                     # gp = Goal Pose

        gp.x = float(input('Enter Goal X coordinate '))
        gp.y = float(input('Enter Goal Y coordinate '))
        distance_tolerance = float(input('Enter Tolerance '))

        I_error = 0
        I_angle_error = 0
        last_error = sqrt(pow((gp.x - self.pose.x), 2) + pow((gp.y - self.pose.y), 2))
        last_angle_error = ((atan2(gp.y - self.pose.y, gp.x - self.pose.x)) - self.pose.theta)
        # print(last_error)

        while (last_error >= distance_tolerance):
            
            error = sqrt(pow((gp.x - self.pose.x), 2) + pow((gp.y - self.pose.y), 2))
            D_error = error - last_error
            I_error = I_error + error
            

            vel_msgs.linear.x = kp*(error) + kd * (last_error) + ki * (I_error)
            vel_msgs.linear.y = 0
            vel_msgs.linear.z = 0
            vel_msgs.angular.x = 0
            vel_msgs.angular.y = 0
            print(last_error)

            angle_error = ((atan2(gp.y - self.pose.y, gp.x - self.pose.x)) - self.pose.theta)
            D_angle_error = angle_error - last_angle_error
            vel_msgs.angular.z = 6*kp * (angle_error) + kd * (D_angle_error) + ki * I_angle_error

            self.velocity_publisher.publish(vel_msgs)
            # print("X pose of turtle is: " , self.pose.x)

            self.rate.sleep()
            last_error = error
            last_angle_error = angle_error
            I_angle_error = I_angle_error + last_angle_error


        vel_msgs.linear.x = 0
        vel_msgs.angular.z = 0
        self.velocity_publisher.publish(vel_msgs)

        rospy.spin()
 
if __name__ == '__main__':
    try:
        x = turtlebot()
        x.moveGoal()


    except rospy.ROSInterruptException: pass
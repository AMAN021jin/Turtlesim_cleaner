#!/usr/bin/env python3

from math import atan2, sqrt, pow, pi
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
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

    def accel_decel(self,final_vel, initial_vel):
        K_accel=0.5
        K_decel=0.9
        if(final_vel.linear.x-initial_vel.linear.x > 0):
            k = K_accel
        else:
            k = K_decel
        initial_vel = Twist()
            
        change = k*(final_vel.linear.x-initial_vel.linear.x)

        change_angular = 4*k*(final_vel.angular.z-initial_vel.angular.z)
        initial_vel.linear.x = initial_vel.linear.x + change
        initial_vel.angular.z = initial_vel.angular.z + change_angular
            
        self.velocity_publisher.publish(initial_vel)
        return (initial_vel)
    
    def rotate(self, angle):

        vel_angle = Twist()
        i=0
        while((abs((angle*pi/180)-(self.pose.theta))) > 0.05):

            vel_angle.angular.z=(angle*pi/180)-(self.pose.theta)
            self.velocity_publisher.publish(vel_angle)

    def moveGoal(self, gp):
        # time = rospy.Time.now().to_sec()

        vel_msgs = Twist()
        last_vel_msgs = Twist()

        I_error = 0
        I_angle_error = 0
        last_error = sqrt(pow((gp.x - self.pose.x), 2) + pow((gp.y - self.pose.y), 2))
        last_angle_error = ((atan2(gp.y - self.pose.y, gp.x - self.pose.x)) - self.pose.theta)

        while (last_error >= 0.1):
            I_error = I_error + last_error
            error = sqrt(pow((gp.x - self.pose.x), 2) + pow((gp.y - self.pose.y), 2))
            D_error = error - last_error
            

            vel_msgs.linear.x = kp*(error) + kd * (last_error) + ki * (I_error)
            vel_msgs.linear.y = 0
            vel_msgs.linear.z = 0
            vel_msgs.angular.x = 0
            vel_msgs.angular.y = 0

            angle_error = ((atan2(gp.y - self.pose.y, gp.x - self.pose.x)) - self.pose.theta)
            D_angle_error = angle_error - last_angle_error
            vel_msgs.angular.z = 6*kp * (angle_error) + kd * (D_angle_error) + ki * I_angle_error

            last_vel_msgs = self.accel_decel(vel_msgs, last_vel_msgs)

            # self.velocity_publisher.publish(vel_msgs)

            self.rate.sleep()
            last_error = error
            last_angle_error = angle_error
            I_angle_error = I_angle_error + last_angle_error


        # vel_msgs.linear.x = 0
        # vel_msgs.angular.z = 0
        # self.velocity_publisher.publish(vel_msgs)

        # rospy.spin()

    def grid(self):

        gp = Pose()
        grid_corner_points=[(1,1,0),(10,1,90),(10,2,180),(1,2,90),(1,4,0),(10,4,90),(10,6,180),(1,6,90),(1,8,0),(10,8,90),(10,10,180),(1,10,0)]
        for i in range(len(grid_corner_points)):
            gp.x=grid_corner_points[i][0]
            gp.y=grid_corner_points[i][1]
            self.moveGoal(gp)
            self.rotate(grid_corner_points[i][2])


if __name__ == '__main__':
    try:
        x = turtlebot()
        x.grid()


    except rospy.ROSInterruptException: pass
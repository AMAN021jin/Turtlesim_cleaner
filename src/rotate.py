#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
# from turtlesim.msgs import Pose
PI = 3.14

angular_speed = float(input("Enter speed"))
angle = float(input("Enter angle"))
clockwise = bool(int(input("Enter clockwise or not")))



def rotate(angular_speed, angle, clockwise):                        #there will be positive angle for anti-clockwise rotation and negative angle for clockwise rotation
    rospy.init_node('robot_rotate', anonymous=True)

    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    angular_speed = angular_speed*2*PI/360
    # print(angular_speed)
    angle = (angle*PI)/180
    print(angle)

    vel_msg.linear.x = 0

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    # vel_msg.angular.z = 2
    # velocity_publisher.publish(vel_msg)

    if (clockwise):
        vel_msg.angular.z = -abs(angular_speed)

    else:
        vel_msg.angular.z = abs(angular_speed)

    current_angle = 0.000000
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        
        
        while (current_angle < angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            # t0 = int(t0)
            # t1 = int(t1)
            current_angle = angular_speed * (t1-t0)
            # print(current_angle)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        rotate(angular_speed, angle, clockwise)

    except rospy.ROSInterruptException: pass
    
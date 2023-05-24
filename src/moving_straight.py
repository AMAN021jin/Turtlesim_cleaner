#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist


speed = float(input("Enter speed"))
distance = float(input("Enter distance"))
isForward = bool(int(input("Enter Direction")))

# speed = input("Enter speed ")
# distance = input("Enter distance ")
# isForward = input("Enter Direction ")

def move (speed, distance, isForward):
    rospy.init_node('robot_cleaner', anonymous=True)

    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()

    if (isForward):
        vel_msg.linear.x = abs(speed)
        print(isForward)
        print(vel_msg.linear)

    else:
        vel_msg.linear.x = -abs(speed)
        print(isForward)
        print(vel_msg.linear)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_distance = 0.000000
    while not rospy.is_shutdown():
        
        
        while (current_distance < distance):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            t0 = int(t0)
            t1 = int(t1)
            current_distance = speed * (t1-t0)
            print(current_distance)

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        move(speed, distance, isForward)

    except rospy.ROSInterruptException: pass    



#!/usr/bin/env python3
"""
In this, we want to set desired orientation of turtle. means :
initially turtle is facing +x when we run turtlesim node then we want to set absolute orientation of tuetle
to thetha degree from +x
so let's see how we can do this!!
I: For this first we need to find the current pose or relative angle of turtle
II: 
"""





import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# turtlesim_pose = Pose()
x=0
y=0
theta=0
def posecallback(pose):
    
    x = pose.x
    y = pose.y
    theta = pose.theta
    # print(theta)
    # rospy.loginfo('Robot is at x = %f : y = %f : theta = %f\n', x,y,theta)

# def move ():
#     rospy.init_node('robot_cleaner', anonymous=True)

#     velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#     pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, posecallback)
#     speed = float(input("Enter speed"))
#     distance = float(input("Enter distance"))
#     isForward = bool(int(input("Enter Direction")))

#     vel_msg = Twist()
# while not rospy.is_shutdown():
#         print(vel_msg.linear)

#     else:
#         vel_msg.linear.x = -abs(speed)
#         print(isForward)
#         print(vel_msg.linear)

#     vel_msg.linear.y = 0
#     vel_msg.linear.z = 0
#     vel_msg.angular.x = 0
#     vel_msg.angular.y = 0
#     vel_msg.angular.z = 0
#     t0 = rospy.Time.now().to_sec()
#     current_distance = 0.000000
#     while (current_distance < distance):
#         velocity_publisher.publish(vel_msg)
#         t1 = rospy.Time.now().to_sec()
#         t0 = int(t0)
#         t1 = int(t1)
#         current_distance = speed * (t1-t0)
#         print(current_distance)

#     vel_msg.linear.x = 0
#     velocity_publisher.publish(vel_msg)

def setDesiredOrientation(desired_angle):
    rospy.init_node('current_pose', anonymous=True)

    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, posecallback)
    # global x, y, theta
    # turtlesim_pose = Pose()
    # desired_angle = float(input("enter angle : "))
    # relative_angle = desired_angle - 
    while not rospy.is_shutdown():
        # theta = turtlesim_pose.theta
        # relative_angle = desired_angle - theta
        print(x)
    # print(pose.theta)
        rospy.spin()
if __name__ == '__main__':
    try:
        setDesiredOrientation(120)
        # move()
        # rotate()


    except rospy.ROSInterruptException: pass   
    

#!/usr/bin/env python3

from math import atan2, sqrt, pow
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
    
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))
        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin()
if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass


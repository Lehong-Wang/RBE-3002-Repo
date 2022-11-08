#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        # TODO
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        # TODO
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        # TODO
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # TODO
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        self.msg_cmd_vel = Twist()
        
        # self.pose = Point()
        
        self.rate = rospy.Rate(10) # 10hz

        # robot pos
        self.px = 0
        self.py = 0
        self.pth = 0
        print("finished init")



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        # TODO
        # Linear velocity
        self.msg_cmd_vel.linear.x = linear_speed
        self.msg_cmd_vel.linear.y = 0
        self.msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        self.msg_cmd_vel.angular.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0
        self.msg_cmd_vel.angular.z = angular_speed
        ### Publish the message
        # TODO
        self.pub.publish(self.msg_cmd_vel)
        self.rate.sleep()
        # print(f"calling send_speed {(linear_speed, angular_speed)}")

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        # Save the initial pose
        initial_px = self.px
        initial_py = self.py
        error = float('inf')
        tolerance = 0.01

        # Send the speed
        self.send_speed(linear_speed, 0)

        # If reached desired distance
        while (error > tolerance):
            error = abs(math.sqrt((self.px - initial_px) ** 2 + (self.py - initial_py) ** 2) - distance)
            print(f"Px: {self.px}, py: {self.py}, init: {(initial_px, initial_py)}")
            print(f"error: {error}")
            self.send_speed(linear_speed, 0)
            rospy.sleep(0.05)
        # Stop the robot
        self.send_speed(0, 0)
        



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        # Save the initial pose
        initial_pth = self.pth
        maped_init_pth = initial_pth + math.pi
        target_pth = (maped_init_pth + angle) % (2*math.pi) - math.pi
        error = float('inf')
        tolerance = 0.1
        
        # If reached desired distance
        while (error > tolerance):
            error = abs(self.pth - target_pth)
            print(f"pth: {self.px}, init: {(initial_pth)}, target: {target_pth}")
            print(f"error: {error}")
            self.send_speed(0, aspeed)
            rospy.sleep(0.05)
        # Stop the robot
        self.send_speed(0, 0)

    

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # TODO
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        self.pth = yaw
        print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        rospy.sleep(1)
        self.rotate(3, -0.2)
        # while not rospy.is_shutdown():
        #     self.send_speed(0,0.5)
        # self.drive(0.3,0.1)
        # while not rospy.is_shutdown():
        # # self.send_speed(0.5, 1)
        #     self.drive(1, 1)
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()

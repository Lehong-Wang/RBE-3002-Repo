#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path

class Drive:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'drive'
        rospy.init_node('drive')
        
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)
        
        
        ## Create subscribers for service call
        self.goal_pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.call_astar)
            
        self.msg_cmd_vel = Twist() # Make a new Twist message
        
        self.rate = rospy.Rate(10) # 10hz

        # Robot pos
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
        # Linear velocity
        self.msg_cmd_vel.linear.x = linear_speed
        self.msg_cmd_vel.linear.y = 0
        self.msg_cmd_vel.linear.z = 0.0
        
        # Angular velocity
        self.msg_cmd_vel.angular.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0
        self.msg_cmd_vel.angular.z = angular_speed
        
        ### Publish the message
        self.pub.publish(self.msg_cmd_vel)
        self.rate.sleep()
        
        # print(f"calling send_speed {(linear_speed, angular_speed)}")

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # Save the initial pose
        initial_px = self.px
        initial_py = self.py
        
        # Create variables
        error = float('inf')
        tolerance = 0.01

        # If reached desired distance
        while (error > tolerance):
            error = distance - math.sqrt((self.px-initial_px)**2 + (self.py-initial_py)**2)
            #print(f"Px: {self.px}, py: {self.py}, init: {(initial_px, initial_py)}")
            #print(f"error: {error}")
            self.send_speed(linear_speed, 0) # Send speed
            rospy.sleep(0.05)
        # Stop the robot
        self.send_speed(0, 0)
        



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # Save the initial pose
        initial_pth = self.pth
        mapped_init_pth = initial_pth + math.pi
        
        # Process input
        aspeed = abs(aspeed)    # speed is positive
        angle = (angle+math.pi) % (2*math.pi) - math.pi   # make angle (-pi, pi)
        # angle = (angle+math.pi) % (2*math.pi)   # make angle (0, 2*pi)
        
        # Go reverse when angle < 0
        if angle < 0:
            aspeed = -aspeed
        print(f"@rotate\t Rotate {angle}")
        
        # Create variables
        target_pth = (mapped_init_pth+angle) % (2*math.pi) - math.pi
        error = float('inf')
        tolerance = 0.05

        # If reached desired distance
        while (error > tolerance):
            error = abs(self.pth - target_pth)
            print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")

            print(f"pth: {self.pth}, init: {(initial_pth)}, target: {target_pth}")
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
        # Save the initial pose
        initial_pth = self.pth
        initial_px = self.px
        initial_py = self.py
        
        # Store the message position
        desired_px = msg.pose.position.x
        desired_py = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        desired_pth = yaw
        print(f"PoseStamped {(round(desired_px,3), round(desired_py,3), round(desired_pth,3))}")
        print(f"Current pose {(round(self.px,3), round(self.py,3), round(self.pth,3))}")
        
        # Calculate the rotation needed based on desired position
        x_change = desired_px - initial_px
        y_change = desired_py - initial_py
        angle = math.atan2(y_change, x_change)
        
        # Call rotate()
        print(f"Rotating {angle - self.pth}")
        self.rotate(angle - self.pth, 0.5)
        self.send_speed(0, 0)
        rospy.sleep(0.5)
        print(f"Current pose {(round(self.px,3), round(self.py,3), round(self.pth,3))}")
        
        # Calculate the distance needed based on desired position
        target_distance = math.sqrt((desired_px-initial_px)**2 + (desired_py-initial_py)**2)
        print(f'target Distnace: {target_distance}')
        
        # Call drive()
        self.drive(target_distance, 0.1)
        self.send_speed(0, 0)
        rospy.sleep(0.5)
        print(f"Current pose {(round(self.px,3), round(self.py,3), round(self.pth,3))}")
        
        # Call rotate for the rest of the rotation needed
        print(f"Rotating {desired_pth - self.pth}")
        self.rotate(desired_pth - self.pth, 0.5)
        self.send_speed(0, 0)
        rospy.sleep(0.5)
        
        print(f"Current pose {(round(self.px,3), round(self.py,3), round(self.pth,3))}")
        print(f"Finished\nInitial{(initial_px, initial_py, initial_pth)}")
        print(f"Final{(self.px, self.py, self.pth)}")
        print(f"Goal {(desired_px, desired_py, desired_pth)}")


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        self.pth = yaw
        # print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")


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
        

    def call_astar(self, msg):
        """
        Creates the path using Astar from Rviz
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The goal pose.
        """
        ## Create service proxy
        path_plan = rospy.ServiceProxy('plan_path', GetPlan)
        initial_pose = PoseStamped()
        
        # Save the initial pose
        initial_pose.pose.position.x = self.px
        initial_pose.pose.position.y = self.py
        initial_pose.pose.orientation.w = self.pth
        
        tolerance = 0.1
        
        send = path_plan(initial_pose, msg, tolerance)
        rospy.loginfo("sent path from rviz")
            
      

    def run(self):
        print("Sleep")
        rospy.sleep(1)
        print("Wake up")
        rospy.spin()
  

if __name__ == '__main__':
    Drive().run()

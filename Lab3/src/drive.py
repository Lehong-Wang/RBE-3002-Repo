#!/usr/bin/env python3

import math
from math import sin, cos
import warnings
import numpy as np


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.pos_to_robot_frame_callback)
        # self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.run_bezier_traj)
        # self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        self.goal_pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.call_astar)


        self.msg_cmd_vel = Twist() # Make a new Twist message

        self.rate = rospy.Rate(50) # 10hz
        self.check_pos_rate = rospy.Rate(20)
        self.check_pos_tolerance = 0.03

        # Robot pos
        self.px = 0
        self.py = 0
        self.pth = 0

        # Timer object, get second with .secs , nanoseconds with .nsecs 
        self.start_timer = rospy.get_rostime()

        print("finished init")




    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """

        # at most any speed, completely off track by 5 m
        # at 0.5 m/s, off by 3m
        # at 0.6 m/s, off by 1m
        # at 0.7 m/s, uncontrolable
        # recomand speed <= 0.4 m/s
        if linear_speed > 0.5:
            warn_str = f"Linear Speed above 0.5 m/s is not recommended.\nCurrent: {linear_speed}"
            warnings.warn(warn_str)
        # Length between wheel is 160 mm
        # 0.4 linear speed -> 5 rad/s
        # at 10 rad/s, some off set, roatation sometimes not stable
        # at 20 rad/s, uncontrolable
        if angular_speed > 10:
            warn_str = f"Angular Speed above 10 rad/s is not recommended.\nCurrent: {angular_speed}"
            warnings.warn(warn_str)

        # Linear velocity
        self.msg_cmd_vel.linear.x = linear_speed
        self.msg_cmd_vel.linear.y = 0.0
        self.msg_cmd_vel.linear.z = 0.0

        # Angular velocity
        self.msg_cmd_vel.angular.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0
        self.msg_cmd_vel.angular.z = angular_speed

        ### Publish the message
        self.pub.publish(self.msg_cmd_vel)
        # print(self.msg_cmd_vel)
        self.rate.sleep()

        # print(f"calling send_speed {(linear_speed, angular_speed)}")



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



################################ trajectory control ################################





    ################################# Original ###############################


        
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
        print(f"Finished Drive {distance}")
        print(f"From: {(initial_px, initial_py)}\tTo: {(self.px, self.py)}")


    def drive_with_correction(self, dest, linear_speed):
        self.run_wave_point_list_pid([dest], linear_speed)


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
        # print(f"@rotate\t Rotate {angle}")
        
        # Create variables
        target_pth = (mapped_init_pth+angle) % (2*math.pi) - math.pi
        error = float('inf')
        tolerance = 0.05

        # If reached desired distance
        while (error > tolerance):
            error = abs(self.pth - target_pth)
            # print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")

            # print(f"pth: {self.pth}, init: {(initial_pth)}, target: {target_pth}")
            # print(f"error: {error}")
            self.send_speed(0, aspeed)
            rospy.sleep(0.05)
        # Stop the robot
        self.send_speed(0, 0)
        print(f"Finished Rotate {angle}")
        print(f"From: {initial_pth}\tTo: {self.pth}")
        


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
        
        # # Calculate the distance needed based on desired position
        # target_distance = math.sqrt((desired_px-initial_px)**2 + (desired_py-initial_py)**2)
        # print(f'target Distnace: {target_distance}')
        
        # # Call drive()
        # self.drive(target_distance, 0.1)
        # self.send_speed(0, 0)
        # rospy.sleep(0.5)
        # print(f"Current pose {(round(self.px,3), round(self.py,3), round(self.pth,3))}")
        
        self.drive_with_correction((desired_px, desired_py), 0.1)

        # Call rotate for the rest of the rotation needed
        print(f"Rotating {desired_pth - self.pth}")
        self.rotate(desired_pth - self.pth, 0.5)
        self.send_speed(0, 0)
        rospy.sleep(0.5)
        
        print(f"Current pose {(round(self.px,3), round(self.py,3), round(self.pth,3))}")
        print(f"Finished\nInitial{(initial_px, initial_py, initial_pth)}")
        print(f"Final{(self.px, self.py, self.pth)}")
        print(f"Goal {(desired_px, desired_py, desired_pth)}")










    ################################# Arc ###############################



    def short_arc(self, pose_x_y, speed):
        """
        drive to target with short arc
        a lot of aprocimation, work better for straighter path
        :param [(x,y)] The target pose.
        :param [m/s] linear speed
        :return [s] total expected time to complete arc
        """
        x2, y2 = pose_x_y

        x1 = self.px
        y1 = self.py
        theta = self.pth

        # aprocimation arc with straight line
        displacement = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        # print(f"displacement {displacement}")

        # angle between start-goal line and horizontal line
        line_angle = math.atan2(y2-y1, x2-x1)
        # angle between start orientation and goal orientation
        # TODO 
        # how to add two angles properly? (-pi, pi)
        rotaional_displacement = 2 * (line_angle - theta)
        print(f"Old rot: {rotaional_displacement}")
        # rotaional_displacement = 2 * self.merge_angle("sub", line_angle, theta)
        # print(f"New rot: {rotaional_displacement}")

        # rotaional_displacement = (rotaional_displacement + math.pi) % (2*math.pi) - math.pi
        # print(f"rotational_displacement {rotaional_displacement}")

        # raduis = (displacement/2) / (math.sin(abs(rotaional_displacement/2)) + 0.001)
        raduis = (displacement/2) / (math.sin(rotaional_displacement/2) + 0.001)
        print(f"raduis {raduis}")
        arc_length = raduis * rotaional_displacement
        print(f"arc_length {arc_length}")
        total_time = arc_length / speed
        angular_speed = rotaional_displacement / total_time
        print(f"speed {(speed, angular_speed)}")
        self.send_speed(speed, angular_speed)
        return total_time




    def run_wave_point_list_arc(self, wave_point_list, speed):
        time_tolerance_factor = 1.2

        for point in wave_point_list:
            expected_time = self.short_arc(point, speed)
            print(f"Expected time: {expected_time}")
            start_t = self.get_time()

            dist_list = []
            dist = math.sqrt((self.px-point[0])**2 + (self.py-point[1])**2)
            dist_list.append(dist)

            while dist > self.check_pos_tolerance:
                current_t = self.get_time()
                elapsed_t = current_t - start_t
                dist = math.sqrt((self.px-point[0])**2 + (self.py-point[1])**2)
                dist_list.append(dist)
                print(dist)
                if elapsed_t > time_tolerance_factor * expected_time:
                    rospy.loginfo(f"Wave point {point} not reached in time, recalculating route.")
                    if dist_list[-1] > dist_list[-2] and dist_list[-2] > dist_list[-3]:
                        break

                self.check_pos_rate.sleep()
            self.send_speed(0,0)
            rospy.sleep(3)


    def check_pos(self, target):
        dist = math.sqrt((self.px-target[0])**2 + (self.py-target[1])**2)
        print(dist)
        return dist < self.check_pos_tolerance



    ################################# PID ###############################


    def pid_control(self, pose_x_y, speed, d_angle):
        x2, y2 = pose_x_y

        x1 = self.px
        y1 = self.py
        theta = self.pth

        kp_angle = 6
        kd_angle = 1

        target_angle = math.atan2(y2-y1, x2-x1)
        p_angle = target_angle - theta

        v_angle = kp_angle * p_angle - kd_angle * d_angle

        v_angle = min(v_angle, 10)

        # self.send_speed(speed, v_angle)
        return (speed, v_angle)




    def run_wave_point_list_pid(self, wave_point_list, speed):

        rospy.loginfo(f"Wave Points: {wave_point_list}\nSpeed: {speed}")
        time_tolerance_factor = 1.2

        for i,point in enumerate(wave_point_list):
            total_dist = math.sqrt((self.px-point[0])**2 + (self.py-point[1])**2)
            expected_time = total_dist / speed
            print(f"Expected time: {expected_time}")
            start_t = self.get_time()

            dist_list = []
            dist = math.sqrt((self.px-point[0])**2 + (self.py-point[1])**2)
            dist_list.append(dist)

            angular_speed_list = [0.0, 0.0, 0.0]
            # angular speed for d controller
            d_angle = 0
            # set a virtrual goal behind actrual goal to pervent angle fluctruation near goal
            virtrual_goal = (point[0] + 0.5*(point[0]-self.px), point[1] + 0.5*(point[1]-self.py))

            while dist > self.check_pos_tolerance:
                current_t = self.get_time()
                elapsed_t = current_t - start_t
                dist = math.sqrt((self.px-point[0])**2 + (self.py-point[1])**2)
                dist_list.append(dist)
                print(dist)
                if elapsed_t > time_tolerance_factor * expected_time:
                    rospy.loginfo(f"Wave point {point} not reached in time, recalculating route.")
                    if dist_list[-1] > dist_list[-2] and dist_list[-2] > dist_list[-3]:
                        break


                linear_speed, angular_speed = self.pid_control(virtrual_goal, speed, d_angle)
                if i == 0:
                    linear_speed, angular_speed = self.smooth_start(linear_speed, angular_speed, dist_list)
                if i == len(wave_point_list) - 1:
                    linear_speed, angular_speed = self.smooth_stop(linear_speed, angular_speed, dist_list)
                self.send_speed(linear_speed, angular_speed)
                angular_speed_list.append(angular_speed)
                d_angle = 1/2 * (angular_speed_list[-1] + angular_speed_list[-2])

                # self.check_pos_rate.sleep()

            self.send_speed(0,0)
            print(f"Reached {point}")
            # rospy.sleep(1)

        rospy.loginfo(f"Goal {wave_point_list[-1]} Reached !")



    def smooth_start(self, linear_speed, angular_speed, dist_list):
        full_start_length = 10
        current_length = len(dist_list)
        if current_length < full_start_length:
            linear_speed = linear_speed * current_length / full_start_length
            print(f"Speed: {linear_speed}")

        return linear_speed, angular_speed


    def smooth_stop(self, linear_speed, angular_speed, dist_list):
        start_break_dist = 0.1
        current_dist = dist_list[-1]
        if current_dist < start_break_dist:
            linear_speed = linear_speed * current_dist*10
            print(f"Speed: {linear_speed}")
        return linear_speed, angular_speed




    ################################### service call #####################################





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

        print(send)
        self.drive_along_path(send)
    


    def drive_along_path(self, msg):
        """
        given a nav_msgs/Path message
        extract wave points
        walk along wave points
        """
        wave_point_list = []
        pose_msg_list = msg.plan.poses
        for pose_msg in pose_msg_list:
            x = pose_msg.pose.position.x
            y = pose_msg.pose.position.y
            wave_point_list.append((x,y))

        print(f"Wave Points: {wave_point_list}")

        self.run_wave_point_list_pid(wave_point_list, 0.1)






    ################################### util #####################################

    def merge_angle(self, operation, th1, th2):
        """
        do operation (+/-) on two angle in range (-pi, pi)
        """

        maped_th1 = th1 + math.pi
        maped_th2 = th2 + math.pi

        if operation == "add":
            ret = (maped_th1 + maped_th2) % math.pi - math.pi
        elif operation == "sub":
            ret = (maped_th1 - maped_th2) % math.pi - math.pi
        else:
            warn_str = f"Merge angle operation not recognized. Choose add or sub\n Given {operation}"
            warnings.warn(warn_str)
            return 0
        return ret


    def get_time(self, now=None):
        """
        Returns the time to 3 digits
        Convert if given ros Timer object
        """
        if not now:
            now = rospy.get_rostime()
        time = now.secs + int(str(now.nsecs)[:3]) * 0.001
        # print(f"get_time: {time}")
        return time


    def pos_to_robot_frame_callback(self, msg):

        x2 = msg.pose.position.x
        y2 = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        th2 = yaw

        self.pos_to_robot_frame(x2, y2, th2)


    def pos_to_robot_frame(self,x,y,th=None):
        if th is None:
            th = 0

        x0 = self.px
        y0 = self.py
        th0 = self.pth

        trans_mat = np.matrix([[x-x0], [y-y0]])
        rot_mat = np.matrix([[cos(-th0), -sin(-th0)],\
                            [sin(-th0), cos(-th0)]])
        robot_frame = np.matmul(rot_mat, trans_mat)
        xr = robot_frame[0,0]
        yr = robot_frame[1,0]

        print(f"\nWorld Frame: {(x,y,th)}")
        print(f"Robot: {(x0,y0,th0)}")
        print(trans_mat)
        print(robot_frame)
        print(f"Robot Frame: {(xr,yr)}")
        return (xr, yr)




    ################################### run #####################################



    def run(self):
        print("Sleep")
        rospy.sleep(1)
        print("Wake up")


        rospy.spin()

if __name__ == '__main__':
    Lab2().run()

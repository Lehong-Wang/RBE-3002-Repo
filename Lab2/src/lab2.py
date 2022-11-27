#!/usr/bin/env python3

import math
from math import sin, cos
import warnings
import numpy as np

from bezier import *
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


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
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.run_bezier_traj)
        # self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

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














    # def arc_to(self, msg):
    #     """
    #     Drives to a given position in an arc.
    #     :param msg [PoseStamped] The target pose.
    #     """
    #     # Store the message position
    #     desired_px = msg.pose.position.x
    #     desired_py = msg.pose.position.y
    #     quat_orig = msg.pose.orientation
    #     quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
    #     ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
    #     desired_pth = yaw

    #     initial_px = self.px
    #     initial_py = self.py
    #     initial_pth = self.pth

    #     self.test_arc((desired_px, desired_py), 10)


    # def test_arc(self, pose, time):
    #     start_t = self.get_time()
    #     self.short_arc(pose, time)
    #     print(f"Start pose: {(self.px,self.py,self.pth)}")

    #     while (self.get_time() < start_t + time):
    #         # print(rospy.get_rostime().secs - self.start_timer.secs)
    #         pass
    #     self.send_speed(0, 0)
    #     print(f"End pose: {(self.px,self.py,self.pth)}\n")


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



    def run_bezier_traj(self, msg):
        # TODO linear speed maye changable? maybe not
        linear_speed = 0.1

        x1 = self.px
        y1 = self.py
        th1 = self.pth
        
        x2 = msg.pose.position.x
        y2 = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        th2 = yaw

        bezier_traj, total_time = self.bezier_traj(x1,y1,th1, x2,y2,th2, linear_speed)

        wave_points = bezier_traj.get_wave_points(10)

        self.run_wave_point_list_pid(wave_points, 0.1)

        # # time here is rounded to 3 digits
        # t0 = self.get_time()
        # t = t0

        # # TODO changed time
        # while (t < t0 + total_time*1.2):
        #     x_t, y_t = bezier_traj.calc_curve(t-t0)
        #     self.short_arc((x_t, y_t), 0.5)
        #     rospy.sleep(0.4)
        #     t = self.get_time()

        # self.send_speed(0,0)





    def bezier_traj(self, x1, y1, th1, x2, y2, th2, linear_speed):
        """
        take in start and end pose (x,y,theta) and linear speed
        return a time dependent bezier trajectory and its total time
        """

        # TODO case of only one control point needed
        # TODO better calculation for control dist for smoother curve

        control_dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        # control point 1 is control_dist alone th1
        p1x = x1 + math.cos(th1) * control_dist
        p1y = y1 + math.sin(th1) * control_dist
        # p2 is dist alone negative th2
        p2x = x2 - math.cos(th2) * control_dist
        p2y = y2 - math.sin(th2) * control_dist

        # TODO replace this with arc length
        total_time = control_dist / linear_speed

        bezier_curve = BezierCurve(x1,y1, p1x,p1y, p2x,p2y, x2,y2, total_time)

        # print(bezier_curve.p0x)
        # print(bezier_curve)
        bezier_curve.plot()

        return (bezier_curve, total_time)










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








    def run(self):
        print("Sleep")
        rospy.sleep(1)
        print("Wake up")

        wave_points = [(0,0), (0.2,0.1), (0.5, 0.7), (1,1), (1.2, 1.2), (1.5, 1.5), (2,2), (1,1)]
        # wave_points = [(0,0), (0.2,0.1),(0.22,0.12),(0.24,0.14),(0.26,0.16),(0.28,0.18),(0.30,0.20),(0.5, 0), (0.5,0.5), (1,1)]
        wave_points = [(0,0), (0.2,0.1), (0.5, 0.7), (1,1)]
        self.run_wave_point_list_pid(wave_points, 0.2)


        # new_timer = rospy.Time.from_sec(0)

        # self.test_arc((-1,1),10)
        # rospy.sleep(1)
        # self.test_arc((1.5,1),20)
        # rospy.sleep(1)
        # self.test_arc((1,-1),10)
        # rospy.sleep(1)

        # self.bezier_traj(1,2,3,4,5,6,7)
        # self.test_arc((0.2,-0.5), 10)

        # while not rospy.is_shutdown():
        #     now = new_timer.now()
        #     # now = rospy.get_rostime()
        #     # rospy.loginfo("Current time", now.secs, now.nsecs)
        #     print("Current time", now.secs, now.nsecs)
        #     print(f"Timer {rospy.get_rostime().secs - self.start_timer.secs}")
        #     rospy.sleep(0.5)

        # self.short_arc((1,-1), 3)

        # self.rotate(-21*math.pi/2, -0.2)
        # while not rospy.is_shutdown():
        #     self.send_speed(0,0.5)
        # self.smooth_drive(3,0.2)
        # while not rospy.is_shutdown():
        # # self.send_speed(0.5, 1)
        #     self.drive(1, 1)
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()

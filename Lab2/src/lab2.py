#!/usr/bin/env python3

import math
import warnings

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
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.run_bezier_traj)
        # self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        self.msg_cmd_vel = Twist() # Make a new Twist message
        
        self.rate = rospy.Rate(10) # 10hz

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
            # print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")

            # print(f"pth: {self.pth}, init: {(initial_pth)}, target: {target_pth}")
            # print(f"error: {error}")
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



################################ trajectory control ################################



    def arc_to(self, msg):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        # Store the message position
        desired_px = msg.pose.position.x
        desired_py = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        desired_pth = yaw

        initial_px = self.px
        initial_py = self.py
        initial_pth = self.pth

        self.test_arc((desired_px, desired_py), 10)


    def test_arc(self, pose, time):
        start_t = self.get_time()
        self.short_arc(pose, time)
        print(f"Start pose: {(self.px,self.py,self.pth)}")

        while (self.get_time() < start_t + time):
            # print(rospy.get_rostime().secs - self.start_timer.secs)
            pass
        self.send_speed(0, 0)
        print(f"End pose: {(self.px,self.py,self.pth)}\n")


    def short_arc(self, pose_x_y, time):
        """
        drive to target with short arc
        a lot of aprocimation, work better for straighter path
        :param [(x,y)] The target pose.
        :param [s] total time in second
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
        rotaional_displacement = 2 * self.merge_angle("sub", line_angle, theta)
        print(f"New rot: {rotaional_displacement}")

        # rotaional_displacement = (rotaional_displacement + math.pi) % (2*math.pi) - math.pi
        # print(f"rotational_displacement {rotaional_displacement}")

        # raduis = (displacement/2) / (math.sin(abs(rotaional_displacement/2)) + 0.001)
        raduis = (displacement/2) / (math.sin(rotaional_displacement/2) + 0.001)
        print(f"raduis {raduis}")
        arc_length = raduis * rotaional_displacement
        print(f"arc_length {arc_length}")
        linear_speed = arc_length / time
        angular_speed = rotaional_displacement / time
        print(f"speed {(linear_speed, angular_speed)}")
        self.send_speed(linear_speed, angular_speed)




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

        # time here is rounded to 3 digits
        t0 = self.get_time()
        t = t0

        # TODO changed time
        while (t < t0 + total_time*1.2):
            x_t, y_t = bezier_traj.calc_curve(t-t0)
            self.short_arc((x_t, y_t), 0.5)
            rospy.sleep(0.4)
            t = self.get_time()

        self.send_speed(0,0)





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







    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # acceleration distance is 0.1m or 40% of total distance
        acc_distance = min(0.5, 0.4*distance)

        initial_px = self.px
        initial_py = self.py
        error = float('inf')
        tolerance = 0.01

        # If reached desired distance
        while (error > tolerance):
            current_distance = math.sqrt((self.px-initial_px) ** 2 + (self.py-initial_py) ** 2)
            error = distance - current_distance
            #print(f"Px: {round(self.px,3)}, py: {round(self.py,3)}, init: {(initial_px, initial_py)}")
            #print(f"error: {error}")

            # fraction of covered distance compared to acc_distance, start and end considered
            dist_fraction = min(current_distance, distance-current_distance)/ acc_distance
            dist_fraction = max(dist_fraction, 0)
            # speed during acceleration is sqrt(dis_frac) * speed + base_speed
            # also smaller than target speed
            acc_speed = min(math.sqrt(dist_fraction) * linear_speed + 0.05, linear_speed)
            self.send_speed(acc_speed, 0)
            rospy.sleep(0.1)
        # Stop the robot
        self.send_speed(0, 0)

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



    def run(self):
        print("Sleep")
        rospy.sleep(1)
        print("Wake up")
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

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty, Int32



class Phase2:

    def __init__(self):

        rospy.init_node("phase_2_node")

        self.init_phase_pub = rospy.Publisher('/task_control/init_phase', Int32, queue_size=10)
        self.request_init_phase_sub = rospy.Subscriber('/task_control/request_init_phase', Empty, self.send_phase)

    def send_phase(self, msg):
        rospy.loginfo("Recieived Phase request, Sending Phase = 2")
        self.init_phase_pub.publish(2)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Phase2().run()
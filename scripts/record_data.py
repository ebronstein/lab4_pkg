#!/usr/bin/env python
"""
Script to record data on the soft finger
Author: Chris Correa
"""
import numpy as np
import pandas as pd
import csv
import os

import rospy
from lab4_pkg.msg import SoftGripperState, SoftGripperCmd

class DataRecorder():
    def __init__(self):
        """
        Records data for the soft finger

        Parameters
        ----------
        """
        self.cmd_pub = rospy.Publisher('soft_gripper_cmd', SoftGripperCmd, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber('soft_gripper_state', SoftGripperState, self.state_listener)
        rospy.on_shutdown(self.shutdown)

        self.states = []
        self.rate = rospy.Rate(100)
        self.start_time = None

    def state_listener(self, msg):
        """
        Records states from the 'soft_gripper_state' topic

        Parameters
        ----------
        msg : :obj:`lab4_pkg.SoftGripperState`
        """
        if self.start_time is None:
            self.start_time = msg.time
        self.states.append([
            msg.time - self.start_time, 
            msg.left_pwm, msg.right_pwm, 
            msg.left_pressure, msg.right_pressure, 
            msg.left_flex, msg.right_flex, 
            msg.base_pos.x, msg.base_pos.y, 
            msg.tip_pos.x, msg.tip_pos.y
        ])

    def flush(self, run_name):
        """
        writes states to file
        """
        df = pd.DataFrame(
            np.array(self.states), 
            columns=[
                'time', 
                'left_pwm', 'right_pwm', 
                'left_pressure', 'right_pressure', 
                'left_flex', 'right_flex', 
                'base_pos_x', 'base_pos_y', 
                'tip_pos_x', 'tip_pos_y'
            ]
        )
        pkg_dir = '/home/cc/ee106b/sp19/class/ee106b-aap/ee106b_sp19/ros_workspaces/lab4_ws/src/lab4_pkg/'                
        filename = os.path.join(pkg_dir, 'data/bend_calibration/', run_name)
        print 'Writing CSV to {0}'.format(filename)
        df.to_csv(filename)
        self.states = [] # flush states
        self.start_time = None

    def record_data(self):
        """
        Script to command soft finger.  You can send commands to both fingers, but only the right is attached.
        """
        num_pwm_vals = 10
        num_samples = 1
        min_pwm = 5
        max_pwm = 150
        for cmd in np.linspace(min_pwm, max_pwm, num_pwm_vals):
            for i in range(num_samples):
                self.cmd_pub.publish(SoftGripperCmd(cmd,cmd))
                rospy.sleep(10)
                self.cmd_pub.publish(SoftGripperCmd(0,0))
                rospy.sleep(3)
                self.flush('{0}_{1}.csv'.format(cmd, i))

    def shutdown(self):
        """
        Stops the finger and flushes whenever you exit
        """
        self.cmd_pub.publish(SoftGripperCmd(0,0))

if __name__ == '__main__':
    rospy.init_node('data_recorder')
    dr = DataRecorder()
    dr.record_data()
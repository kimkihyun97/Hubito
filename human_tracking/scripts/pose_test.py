#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from human_tracking.msg import filtered_data
import matplotlib.pyplot as plt
import numpy as np

# 센서 데이터를 저장하는 리스트
robot_z = []
robot_x = []
robot_t = []
person_z = []
person_x = []

def callback_filtered_pose_robot(data):
    robot_z.append(data.rz / 1000)
    robot_x.append(data.rx / 1000)
    robot_t.append(data.ro)  # Save the robot's orientation
    person_z.append(data.hz / 1000)
    person_x.append(data.hx / 1000)

    # robot 데이터가 업데이트 될 때마다 plot 업데이트
    plt.figure('Robot and Human Pose')
    plt.clf()
    plt.scatter(robot_z, robot_x, c='r', alpha=0.3)
    plt.scatter(robot_z[-1], robot_x[-1], c='r', marker='X', label='Latest Robot Position')
    
    if len(person_x) > 0 and len(person_z) > 0:
        plt.scatter(person_z, person_x, c='b', alpha=0.3)
        plt.scatter(person_z[-1], person_x[-1], c='b', marker='X', label='Latest Human Position')

    # Draw a quiver plot for robot orientation on the same graph
    if len(robot_t) > 0:
        plt.quiver(robot_z[-1], robot_x[-1], np.cos(robot_t[-1]), np.sin(robot_t[-1]), angles='xy', scale_units='xy', scale=10, color='r', width=0.003)

    plt.legend(loc='upper right',bbox_to_anchor=(1.5, 1))
    plt.grid(True)
    plt.draw()
    plt.pause(0.001)

def listener():
    rospy.init_node('data_listener', anonymous=True)
    rospy.Subscriber("filtered_data", filtered_data, callback_filtered_pose_robot)

    plt.ion()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    listener()


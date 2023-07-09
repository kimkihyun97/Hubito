#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nlink_parser.msg import LinktrackAoaNodeframe0
import matplotlib.pyplot as plt
import numpy as np

# 데이터를 저장할 리스트 초기화
time_values = []
dis_values = []
angle_values = []
fp_rssi_values = []
rx_rssi_values = []
imu_values = []
imu_time_values = []

start_time = None
update_count = 0

def callback(data):
    global update_count, start_time
    if not start_time:
        start_time = rospy.get_time()
    current_time = rospy.get_time() - start_time
    time_values.append(current_time)

    if data.nodes:
        for node in data.nodes:
            dis_values.append(node.dis)
            angle_values.append(node.angle)
            fp_rssi_values.append(node.fp_rssi)
            rx_rssi_values.append(node.rx_rssi)
    else:
        return

    update_count += 1
    if update_count % 50 != 0:  # Only update the plot every 10 messages
        return

    dis_mean = np.mean(dis_values) if dis_values else None
    dis_var = np.var(dis_values) if dis_values else None

    angle_mean = np.mean(angle_values) if angle_values else None
    angle_var = np.var(angle_values) if angle_values else None

    imu_mean = np.mean(imu_values) if imu_values else None
    imu_var = np.var(imu_values) if imu_values else None

    with open("data.txt", "a") as file:
        file.write("Time: {}, Dis Mean: {}, Dis Var: {}\n".format(current_time, dis_mean, dis_var))
        file.write("Time: {}, Angle Mean: {}, Angle Var: {}\n".format(current_time, angle_mean, angle_var))
        file.write("Time: {}, IMU Mean: {}, IMU Var: {}\n".format(current_time, imu_mean, imu_var))

    plt.clf()

    plt.subplot(4, 1, 1)
    plt.plot(time_values, dis_values, color='red', label='Dis')  # Red color
    plt.title('Dis')
    plt.xlabel('Time(s)')
    plt.ylabel('Dis(m)')
    plt.ylim(0, 1.5)
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(time_values, angle_values, color='blue', label='Angle')  # Blue color
    plt.title('Angle')
    plt.xlabel('Time(s)')
    plt.ylabel('Angle(degree)')
    plt.ylim(0, 90)
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(time_values, np.abs(np.array(fp_rssi_values) - np.array(rx_rssi_values)), color='green')  # Green color
    plt.title('Difference between FP RSSI and RX RSSI')
    plt.xlabel('Time(s)')
    plt.ylabel('RSSI Diff')
    plt.ylim(0, 10)
    plt.grid(True)


    # 간격 조절
    plt.tight_layout(pad=2)

    # 그래프 그리고 업데이트
    plt.pause(0.001)
    plt.draw()

def listener():
    rospy.init_node('listener', anonymous=True)
    global start_time
    start_time = rospy.get_time()
    rospy.Subscriber("nlink_linktrack_aoa_nodeframe0", LinktrackAoaNodeframe0, callback)
    plt.ion()
    plt.show(block=False)
    rospy.spin()

if __name__ == '__main__':
    listener()


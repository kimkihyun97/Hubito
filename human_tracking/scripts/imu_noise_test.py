#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np

# 데이터를 저장할 리스트 초기화
imu_time_values = []
imu_values = []

start_time = None
update_count = 0

def imu_callback(data):
    global start_time, update_count
    if not start_time:
        start_time = rospy.get_time()
    current_time = rospy.get_time() - start_time
    imu_time_values.append(current_time)
    imu_values.append(-data.angular_velocity.y)  # Or use other values based on your requirement

    update_count += 1
    if update_count % 10 != 0:  # Only update the plot every 10 messages
        return

    imu_mean = np.mean(imu_values) if imu_values else None
    imu_var = np.var(imu_values) if imu_values else None

    with open("imu_data.txt", "a") as file:
        file.write("Time: {}, IMU Mean: {}, IMU Var: {}\n".format(current_time, imu_mean, imu_var))

    plt.clf()

    plt.subplot(1, 1, 1)
    plt.plot(imu_time_values, imu_values, color='purple', label='IMU')  # Purple color
    plt.title('IMU')
    plt.xlabel('Time(s)')
    plt.ylabel('angular velocity')
    plt.ylim(-0.1, 0.1)
    plt.grid(True)
    plt.legend()

    # 간격 조절
    plt.tight_layout(pad=2)

    # 그래프 그리고 업데이트
    plt.pause(0.001)
    plt.draw()

def listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("camera/imu", Imu, imu_callback)
    plt.ion()
    plt.show(block=False)
    rospy.spin()

if __name__ == '__main__':
    listener()


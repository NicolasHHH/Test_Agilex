#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import time

# CSV file name
imu_csv_file = "imu_log_{}.csv".format(time.strftime("%Y%m%d_%H%M%S", time.localtime()))
gps_csv_file = "gps_log_{}.csv".format(time.strftime("%Y%m%d_%H%M%S", time.localtime()))


# Initializing the CSV file with headers
with open(imu_csv_file, mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(["time",
                     "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                     "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
                     "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"])

with open(gps_csv_file, mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(["time", "latitude", "longitude", "altitude", "position_covariance", "position_covariance_type"])


# Initialization of the time offset variable
imu_time_offset = None
gps_time_offset = None

def imu_callback(data):
    global imu_time_offset
    # Convert ROS timestamp to Unix epoch time
    time_stamp = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs

    # If this is the first message, calculate the time offset
    if imu_time_offset is None:
        imu_time_offset = time.time() - time_stamp

    # Add the time offset to the message timestamp
    time_stamp += imu_time_offset

    with open(imu_csv_file, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([float(time_stamp),
                         data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w,
                         data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                         data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])

def gps_callback(data):
    global gps_time_offset
    # Convert ROS timestamp to Unix epoch time
    time_stamp = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs

    # If this is the first message, calculate the time offset
    if gps_time_offset is None:
        gps_time_offset = time.time() - time_stamp

    # Add the time offset to the message timestamp
    time_stamp += gps_time_offset

    with open(gps_csv_file, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([float(time_stamp),
                         data.latitude, data.longitude, data.altitude] +
                         list(data.position_covariance) + [data.position_covariance_type])

def listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.Subscriber("gps/fix", NavSatFix, gps_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    with open(gps_csv_file, mode='r') as file:
        lines = file.readlines()
    for line in lines[:3]:
        print(line[4], type(line[4]))
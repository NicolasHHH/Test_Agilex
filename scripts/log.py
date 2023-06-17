#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.msg import ModelStates  # ground truth
import time

# CSV file name
imu_csv_file = "imu_log_{}.csv".format(time.strftime("%Y%m%d_%H%M%S", time.localtime()))
gps_csv_file = "gps_log_{}.csv".format(time.strftime("%Y%m%d_%H%M%S", time.localtime()))
gms_csv_file = "gms_log_{}.csv".format(time.strftime("%Y%m%d_%H%M%S", time.localtime()))

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

with open(gms_csv_file, mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(["time", "position_x", "position_y", "position_z",
                     "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                     "linear_velocity_x", "linear_velocity_y", "linear_velocity_z",
                     "angular_velocity_x", "angular_velocity_y", "angular_velocity_z"])

# Initialization of the time offset variable
imu_time_offset = None
gps_time_offset = None
gms_time_offset = None

# Subscriptions
imu_data = None
gps_data = None
gms_data = None


def imu_callback(data):
    global imu_data
    imu_data = data


def gps_callback(data):
    global gps_data
    gps_data = data


def gms_callback(data):
    global gms_data
    gms_data = data


def imu_timer_callback(event):
    global imu_data
    global imu_time_offset
    if imu_data is None:
        return
    # Convert ROS timestamp to Unix epoch time
    data = imu_data
    imu_data = None
    time_stamp = data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs

    # If this is the first message, calculate the time offset
    if imu_time_offset is None:
        imu_time_offset = time.time() - time_stamp

    # Add the time offset to the message timestamp
    time_stamp += imu_time_offset
    print("imu : ", time_stamp)

    with open(imu_csv_file, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([float(time_stamp),
                         data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w,
                         data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                         data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])


def gps_timer_callback(event):
    global gps_data
    global gps_time_offset
    if gps_data is None:
        return

    data = gps_data
    gps_data = None
    # Convert ROS timestamp to Unix epoch time
    time_stamp = data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs

    # If this is the first message, calculate the time offset
    if gps_time_offset is None:
        gps_time_offset = time.time() - time_stamp

    # Add the time offset to the message timestamp
    time_stamp += gps_time_offset
    print("gps : ", time_stamp)

    with open(gps_csv_file, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([float(time_stamp),
                         data.latitude, data.longitude, data.altitude] +
                        list(data.position_covariance) + [data.position_covariance_type])


def gms_timer_callback(event):
    global gms_data
    global gms_time_offset
    if gms_data is None:
        return
    data = gms_data
    gms_data = None
    robot_index = data.name.index("scout/")
    # Get the pose and twist of the robot
    pose = data.pose[robot_index]
    twist = data.twist[robot_index]

    global gms_time_offset
    # Convert ROS timestamp to Unix epoch time
    time_stamp = rospy.Time.now().to_sec()

    # If this is the first message, calculate the time offset
    if gms_time_offset is None:
        gms_time_offset = time.time() - time_stamp

    # Add the time offset to the message timestamp
    time_stamp += gms_time_offset
    print("state : ", time_stamp)

    with open(gms_csv_file, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([float(time_stamp),
                         pose.position.x, pose.position.y, pose.position.z,
                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                         twist.linear.x, twist.linear.y, twist.linear.z,
                         twist.angular.x, twist.angular.y, twist.angular.z])


def listener():
    rospy.init_node('imu_listener', anonymous=True)

    # Set the rate for each callback
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.Subscriber("gps/fix", NavSatFix, gps_callback)
    rospy.Subscriber("gazebo/model_states", ModelStates, gms_callback, queue_size=1)

    # Set the rate for each callback
    rospy.Timer(rospy.Duration(0.1), imu_timer_callback)
    rospy.Timer(rospy.Duration(0.1), gps_timer_callback)
    rospy.Timer(rospy.Duration(0.1), gms_timer_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

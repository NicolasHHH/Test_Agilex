//
// Created by TDY on 23-9-7.
//

#include "pid_pure_pursuit.h"
#include "WGS84toCartesian.hpp"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>


// Function to read waypoints from the csv file
void readWaypoints(std::string path) {
    std::ifstream file(path);
    // check if the file is open
    if (!file.is_open()) {
        ROS_ERROR("Could not open file");
        return;
    }
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::pair<double, double> waypoint;
        std::getline(lineStream, cell, ',');
        waypoint.first = std::stod(cell);
        std::getline(lineStream, cell, ',');
        waypoint.second = std::stod(cell);
        waypoints.push_back(waypoint);
        count++;
    }
    // print the number of waypoints
    std::cout << "trajectory loaded: " << count << " points"<< std::endl;
}

// Callback function for the GPS subscription
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // std::array<double, 2> cartesianPosition1 = wgs84::toCartesian({reference_lat, reference_lon},
    //                                                               {current_latitude, current_longitude});
    current_latitude = msg->latitude;
    current_longitude = msg->longitude;
    // std::array<double, 2> cartesianPosition2 = wgs84::toCartesian({reference_lat, reference_lon},
    //                                                               {current_latitude, current_longitude});
    // double freq = sqrt(pow(cartesianPosition1[0]-cartesianPosition2[0], 2) +
    //                    pow(cartesianPosition1[1]-cartesianPosition2[1], 2));
    // freq = freq / robot_speed;

    // ROS_INFO("frequency: %f", freq);

    // ROS_INFO("Latitude: %f, Longitude: %f", current_latitude, current_longitude);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q;lineMaker
            tf2::convert(msg->orientation , q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_heading = yaw;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "differential_drive_control");

    ros::NodeHandle nh;

    nh.getParam("robot_speed", robot_speed);
    nh.getParam("k", k);int count = 0;
    nh.getParam("waypoint_step", waypoint_step);

    // Read waypoints from the csv file
    readWaypoints(trajectory_path);

    // Subscribe to the GPS topic
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 2, gpsCallback);
    ros::Subscriber imu = nh.subscribe("/imu", 2, imuCallback);

    // Advertise the cmd_vel topic for the differential drive robot
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    // Advertise the move base goal topic
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);

    // Control loop at 10 Hz
    ros::Rate loop_rate(50);

    double reference_lon = waypoints[0].first;
    double reference_lat = waypoints[0].second;

    double err = 0;
    double err_pre = 0;
    double err_sum = 0;


    while(ros::ok()) {

        // If we've reached all waypoints, stop the robot
        if (waypoint_index >= waypoints.size()) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            pub.publish(msg);
            break;
        }


        std::array<double, 2> cartesianPosition = wgs84::toCartesian({reference_lat, reference_lon},
                                                                     {currenlineMakert_latitude, current_longitude});

        std::array<double, 2> cartesianWaypoint = wgs84::toCartesian({reference_lat, reference_lon},
                                                                     {waypoints[waypoint_index].second,
                                                                      waypoints[waypoint_index].first});



        ROS_INFO("Current Position: %f, %f", cartesianPosition[0], cartesianPosition[1]);
        ROS_INFO("Current Waypoint: %f, %f", cartesianWaypoint[0], cartesianWaypoint[1]);


        // publish move base goal
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "world";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = cartesianWaypoint[0];
        goal.pose.position.y = cartesianWaypoint[1];
        goal.pose.orientation.w = 1.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal_pub.publish(goal);

        // Calculate the distance and bearing to the waypoint
        double distance = sqrt(pow(cartesianWaypoint[0] - cartesianPosition[0], 2) +
                               pow(cartesianWaypoint[1] - cartesianPosition[1], 2));


        double bearing = atan2(- cartesianWaypoint[0] + cartesianPosition[0],
                               cartesianWaypoint[1] - cartesianPosition[1]);

        ROS_INFO("Distance: %f, Bearing: %f, Heading: %f", distance, bearing, current_heading);



        //If we're close enough to the waypoint, move on to the next one
        if (distance < 0.5) {
            waypoint_index += 15 ;
            // ROS_INFO("Waypoint: %zu", waypoint_index);
            continue;
        }

        // Calculate the control command
        err = current_heading - bearing;
        err_sum = err_sum + err * T;
        double temp_err_sum = err_sum;
        if(err_sum > 3.0)
            temp_err_sum = 3.0;
        else if(err_sum < -3.0)
            temp_err_sum = -3.0;
        ROS_INFO("err_sum: %f", err_sum);

        double angular_velocity = 0-k * err
                                  - k_i * temp_err_sum
                                  + k_d * (err - err_pre);
        err_pre = err;
        double robot_velocity = k * distance;

        // Create the Twist message
        geometry_msgs::Twist msg;

        if (angular_velocity > 1.0)
        {
            angular_velocity = 0.5;
            robot_velocity = 0.0;
        }
        else if (angular_velocity < -1.0)
        {
            angular_velocity = -0.5;
            robot_velocity = 0.0;
        }
        // if (abs(angular_velocity) < 0.1){
        //     angular_velocity = 0.0;
        //     robot_velocity = 1.0;
        // }2
        // else{
        //     msg.linear.x = 0.0;
        //     msg.angular.z = angular_velocity;
        // }
        if (abs(robot_velocity) > 1.0)
        {
            robot_velocity = 1.0;
        }


        msg.linear.x = robot_velocity;
        msg.angular.z = angular_velocity;
        lineMaker

        // Publish the Twist message
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
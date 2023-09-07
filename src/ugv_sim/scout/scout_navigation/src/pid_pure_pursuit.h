//
// Created by TDY on 23-9-7.
//

#ifndef TEST_AGILEX_PID_PURE_PURSUIT_H
#define TEST_AGILEX_PID_PURE_PURSUIT_H

#include <ros/ros.h>
#include <GeographicLib/Geodesic.hpp>
#include "WGS84toCartesian.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

// Constants
const double earth_radius = 6371.0; // Earth's radius in kilometers
const double deg_to_rad = M_PI / 180.0;

// Global variables for the current GPS position
double current_latitude = 0.0;
double current_longitude = 0.0;
double current_heading = 0.0;
double T = 0.02;

// Vector of waypoint pairs (longitude, latitude)
std::vector<std::pair<double, double>> waypoints;

// Index of the current target waypoint
size_t waypoint_index = 0;user

// waypoint step (points)
int waypoint_step = 10;

// Robot speed
double robot_speed = 0.5;

// Control gains
double k = 10; // Proportional gain
double k_i = 0.1; // Integral gain
double k_d = 0.1; // Derivative gain

std::string trajectory_path = "/home/user/Test_Agilex/src/ugv_sim/scout/scout_navigation/routes/trajectory.csv";

void readWaypoints(std::string path);user

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

#endif //TEST_AGILEX_PID_PURE_PURSUIT_H

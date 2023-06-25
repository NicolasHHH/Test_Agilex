//
// Created by hty on 24/06/23.
//

#ifndef LINEMARKER_DIFF_PURE_PURSUIT_H
#define LINEMARKER_DIFF_PURE_PURSUIT_H

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

// Vector of waypoint pairs (longitude, latitude)
std::vector<std::pair<double, double>> waypoints;

// Index of the current target waypoint
size_t waypoint_index = 0;

// Robot speed
double robot_speed = 0.5;

// Control gains
double k = 0.5; // Proportional gain

std::string trajectory_path = "/home/hty/LineMarker/Test_Agilex/src/ugv_sim/scout/scout_navigation/routes/trajectory.csv";

void readWaypoints(std::string path);

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

#endif //LINEMARKER_DIFF_PURE_PURSUIT_H

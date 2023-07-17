//
// Created by hty on 17/07/23.
//

#ifndef LINEMARKER_LQR_CONTROL_H
#define LINEMARKER_LQR_CONTROL_H


#include <ros/ros.h>
#include <GeographicLib/Geodesic.hpp>
#include "WGS84toCartesian.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/Dense>
#include <cmath>


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

// waypoint step (points)
int waypoint_step = 10;

// Robot speed
double robot_speed = 0.5;

// Control gains
double k = 0.5; // Proportional gain

//the number of iterations
const double N = 100;
//the expected precision
const double EPSILON = 0.0001;

const double L = 0.590707;

std::string trajectory_path = "/home/hty/LineMarker/Test_Agilex/src/ugv_sim/scout/scout_navigation/routes/trajectory.csv";

void readWaypoints(std::string path);

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

Eigen::MatrixXd lqr(const std::array<double, 2>& cartesianPosition, const std::array<double, 2>& cartesianWaypoint,
                    const double& current_heading,const double& next_robot_speed, const double& bearing);


#endif //LINEMARKER_LQR_CONTROL_H

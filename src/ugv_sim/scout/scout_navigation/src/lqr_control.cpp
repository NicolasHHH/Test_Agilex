//
// Created by hty on 17/07/23.
//

#include "lqr_control.h"


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
    current_latitude = msg->latitude;
    current_longitude = msg->longitude;
    ROS_INFO("Latitude: %f, Longitude: %f", current_latitude, current_longitude);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q;
    tf2::convert(msg->orientation , q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_heading = yaw;
}


//the function to calculate the control variables in LQR, implemented in iteration ways
Eigen::MatrixXd lqr(const std::array<double, 2>& cartesianPosition, const std::array<double, 2>& cartesianWaypoint,
                    const double& current_heading, const double& next_robot_speed, const double& bearing)
{
    //calculate the state equation
    Eigen::MatrixXd A(3, 3);
    Eigen::MatrixXd B(3, 2);
    //the matrix Q and R
    Eigen::MatrixXd Q(3, 3);
    Q << 1, 0, 0,
         0, 1, 0,
         0, 0, 3;
    Eigen::MatrixXd R(2, 2);
    R << 2, 0,
         0, 1;
    double cos_yaw = std::cos(current_heading);
    double sin_yaw = std::sin(current_heading);
    double minus_v_sin_yaw = 0 - next_robot_speed * sin_yaw;
    double v_cos_yaw = next_robot_speed * cos_yaw;
    A << 0, 0, minus_v_sin_yaw,
         0, 0, v_cos_yaw,
         0, 0, 0;
    B << cos_yaw, 0,
         sin_yaw, 0,
         0, 1;
    Eigen::MatrixXd X(3, 1);
    X << cartesianPosition[0] - cartesianWaypoint[0],
         cartesianPosition[1] - cartesianWaypoint[1],
         current_heading - bearing;
    Eigen::MatrixXd P(3, 3);

    P <<    1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    ROS_INFO("B: %f, %f, %f, %f, %f, %f",
             B(0, 0), B(0, 1),
             B(1, 0), B(1, 1),
             B(2, 0), B(2, 1));
    ROS_INFO("A: %f, %f, %f, %f, %f, %f, %f, %f, %f",
                A(0, 0), A(0, 1), A(0, 2),
                A(1, 0), A(1, 1), A(1, 2),
                A(2, 0), A(2, 1), A(2, 2));
    ROS_INFO("R: %f, %f, %f, %f",
             R(0, 0), R(0, 1),
             R(1, 0), R(1, 1));

    Eigen::MatrixXd K;
    for (int i = 0; i < N; i++) {
        K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        Eigen::MatrixXd P_new = Q + A.transpose() * P * (A - B * K); // + K.transpose() * R * K;
        Eigen::MatrixXd diff = P_new - P;
        ROS_INFO("Diff norm: %f", diff.norm());
        if(diff.norm() < EPSILON)
            ROS_INFO("Iteration: %d", i);
            break;
        P = P_new;
    }
    ROS_INFO("P: %f, %f, %f, %f, %f, %f, %f, %f, %f",
             P(0, 0), P(0, 1), P(0, 2),
             P(1, 0), P(1, 1), P(1, 2),
             P(2, 0), P(2, 1), P(2, 2));
    ROS_INFO("K: %f, %f, %f, %f, %f, %f",
             K(0, 0), K(0, 1), K(0, 2),
             K(1, 0), K(1, 1), K(1, 2));
    ROS_INFO("X: %f, %f, %f", X(0, 0), X(1, 0), X(2, 0));

    Eigen::MatrixXd u = K*X;
    return u;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "differential_drive_control");
    ros::NodeHandle nh;
    nh.getParam("robot_speed", robot_speed);
    nh.getParam("k", k);
    nh.getParam("waypoint_step", waypoint_step);
    readWaypoints(trajectory_path);

    // Subscribe to the GPS topic
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 2, gpsCallback);
    ros::Subscriber imu = nh.subscribe("/imu", 2, imuCallback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 5);

    ros::Rate loop_rate(50);
    double reference_lon = waypoints[0].first;
    double reference_lat = waypoints[0].second;
    double next_robot_speed = robot_speed;
    double angular_velocity = 0;

    while(ros::ok()) {
        // If we've reached all waypoints, stop the robot
        if (waypoint_index >= waypoints.size()) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            pub.publish(msg);
            break;
        }
        std::array<double, 2> cartesianPosition =
                wgs84::toCartesian({reference_lat, reference_lon},
                                   {current_latitude, current_longitude});
        std::array<double, 2> cartesianWaypoint =
                wgs84::toCartesian({reference_lat, reference_lon},
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
        double distance = sqrt(pow(cartesianWaypoint[0] -
                                   cartesianPosition[0], 2) +
                               pow(cartesianWaypoint[1] -
                                   cartesianPosition[1], 2));
        double bearing = atan2(-cartesianWaypoint[0] + cartesianPosition[0],
                               cartesianWaypoint[1] - cartesianPosition[1]);

        // If we're close enough to the waypoint, move on to the next one
        if (distance < 0.5) {
            waypoint_index += 30 ;
            ROS_INFO("Waypoint: %zu", waypoint_index);
            continue;
        }
        // Calculate the control command
        ROS_INFO("Distance: %f, Bearing: %f, Heading: %f", distance, bearing, current_heading);
        Eigen::MatrixXd u = lqr(cartesianPosition, cartesianWaypoint,
                                current_heading, next_robot_speed, bearing);
        next_robot_speed = u(0,0) + next_robot_speed;
        angular_velocity = u(1,0) + angular_velocity;
        ROS_INFO("Speed: %f, Angular Velocity: %f", next_robot_speed, angular_velocity);

        // Create the Twist message
        geometry_msgs::Twist msg;
        msg.angular.z = angular_velocity;
        msg.linear.x = next_robot_speed;

        // Publish the Twist message
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
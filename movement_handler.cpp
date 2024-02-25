#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <std_msgs/Int32MultiArray.h>
#include "convert.h"  
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h> 
#include <cmath>
#include <pthread.h>

using namespace UNITREE_LEGGED_SDK;

// Global variables for managing state and goal
ros::Subscriber sub_traffic_light;
double goal_x = 0.0;
double goal_y = 0.0;
ros::Publisher pub_low;
ros::Subscriber sub_odom_info;
ros::Publisher pub_movement;
bool green_activated = false;
bool path_activated = false;
bool goal_reached = false;
int traffic_light_size = 0;

// Callback to update the robot's movement based on traffic light signal
void trafficLightCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data[0] == 1 && green_activated == false) {
        green_activated = true;
        traffic_light_size = msg->data[1];
    }
}

// Calculate the yaw speed required to turn the robot towards the goal
float calculateYawSpeed(const nav_msgs::Odometry &currentOdom) {
    // Compute direction towards the goal and current orientation
    float robotX = currentOdom.pose.pose.position.x;
    float robotY = currentOdom.pose.pose.position.y;
    float goalDirection = atan2(goal_y - robotY, goal_x - robotX);
    tf::Quaternion q(
        currentOdom.pose.pose.orientation.x,
        currentOdom.pose.pose.orientation.y,
        currentOdom.pose.pose.orientation.z,
        currentOdom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // Extract yaw from quaternion

    // Adjust yaw speed based on the difference to the goal direction
    float yawDiff = goalDirection - yaw;
    while (yawDiff > M_PI) yawDiff -= 2 * M_PI;
    while (yawDiff < -M_PI) yawDiff += 2 * M_PI;
    return 1.0 * yawDiff;
}

// Main callback for odometry updates, controls robot's movement
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, Custom* custom) {
    // Extract current position
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // Activate path following when green light is detected
    if (green_activated && !path_activated && !goal_reached) {
        path_activated = true;
        // Calculate goal based on traffic light size
        double distance_to_goal = 0.00136 * traffic_light_size;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double yaw = tf::getYaw(q);
        goal_x = current_x + distance_to_goal * cos(yaw);
        goal_y = current_y + distance_to_goal * sin(yaw);
    }

    // Initialize a command message for movement
    unitree_legged_msgs::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = 0;
    high_cmd_ros.bodyHeight = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;

    // Stop the robot if green light is not activated
    if(!green_activated) {
        pub_movement.publish(high_cmd_ros);
    }

    // Control robot movement towards the goal if path is activated
    if (path_activated && !goal_reached) {
        float distance = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
        if(distance < 1.0) {
            goal_reached = true; // Stop the robot when goal is reached
            pub_movement.publish(high_cmd_ros);
        } else {
            // Configure movement towards the goal
            high_cmd_ros.mode = 2; // Walking mode
            high_cmd_ros.gaitType = 1; // Default gait
            high_cmd_ros.velocity[0] = 1.0f; // Forward speed
            high_cmd_ros.yawSpeed = calculateYawSpeed(*msg); // Yaw speed towards goal
            high_cmd_ros.footRaiseHeight = 0.1; // Adjust foot raise height for walking
            pub_movement.publish(high_cmd_ros);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ptl_navigation");
    ros::NodeHandle nh;

    // Setup ROS subscribers and publishers
    sub_traffic_light = nh.subscribe<std_msgs::Int32MultiArray>("traffic_light_state", 1, trafficLightCallback);
    sub_odom_info = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    pub_movement = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    // Main loop
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce(); // Handle ROS callbacks
        loop_rate.sleep(); // Maintain loop rate
    }

    return 0;
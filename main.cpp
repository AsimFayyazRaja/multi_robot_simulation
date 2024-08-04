#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "robots.h"
#include "map.h"
#include "lidar.h"
#include "visualize.h"

// Map and visualization images
cv::Mat originalMap;
cv::Mat visualizationMap;
cv::Mat grayscaleMap;

// Lidar and robot parameters
int grayThreshold = 230;
std::vector<sensor_msgs::LaserScan> laserScans;
std::vector<bool> draggingRobots;
std::vector<geometry_msgs::Twist> robotVelocities;
geometry_msgs::Twist robotVelocity;
double zoomLevel = 1.0;
bool isDragging = false;

// Configuration and robot state
YAML::Node yamlConfig;
MapConfig mapConfig;
std::vector<Robot> robotList;
std::string selectedRobot;

// Handle mouse events for robot dragging and zooming
void mouseHandler(int event, int x, int y, int flags, void* param) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // Start dragging robots when left mouse button is pressed
        for (size_t i = 0; i < robotList.size(); ++i) {
            cv::Point2d robotPos = worldToImage(robotList[i].pose, mapConfig);
            double dist = std::sqrt(std::pow(x - robotPos.x, 2) + std::pow(y - robotPos.y, 2));
            if (dist <= robotList[i].dimensions.radius / mapConfig.resolution) {
                draggingRobots[i] = true;
            }
        }
    } else if (event == cv::EVENT_MOUSEMOVE) {
        // Update robot positions while dragging
        for (size_t i = 0; i < robotList.size(); ++i) {
            if (draggingRobots[i]) {
                robotList[i].pose = imageToWorld(cv::Point2d(x, y), mapConfig);
                robotList[i].pose.x = std::max(robotList[i].pose.x, -(0.5 * mapConfig.width));
                robotList[i].pose.y = std::max(robotList[i].pose.y, -(0.5 * mapConfig.height));
                robotList[i].pose.x = std::min(robotList[i].pose.x, (0.5 * mapConfig.width));
                robotList[i].pose.y = std::min(robotList[i].pose.y, (0.5 * mapConfig.height));
            }
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        // Stop dragging robots
        for (size_t i = 0; i < robotList.size(); ++i) {
            draggingRobots[i] = false;
        }
    } else if (event == cv::EVENT_MOUSEWHEEL) {
        // Zoom in/out
        if (flags > 0) {
            zoomLevel *= 1.2;
        } else {
            zoomLevel *= 0.8;
        }
        cv::resizeWindow("multi_xy_robot_simulator", static_cast<int>(mapConfig.width * zoomLevel), static_cast<int>(mapConfig.height * zoomLevel));
    }
}

// Callback for receiving velocity commands
void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocityMsg) {
    robotVelocity.linear.x = velocityMsg->linear.x;
    robotVelocity.linear.y = velocityMsg->linear.y;
    robotVelocity.angular.z = velocityMsg->angular.z;
}

// Callback for receiving goal positions
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(goalMsg->pose.orientation, orientation);
    double angle = tf::getYaw(orientation);

    // Draw goal position and orientation
    cv::Point2d goalPosition = worldToImage({goalMsg->pose.position.x, goalMsg->pose.position.y, 0}, mapConfig);
    visualizationMap = drawArrow(visualizationMap, goalPosition, angle, (0, 0, 0), 5);
}

// Setup ROS publishers and subscribers
void setupROS(ros::NodeHandle& nodeHandle, std::vector<ros::Publisher>& posePublishers, 
              std::vector<ros::Publisher>& scanPublishers, std::vector<ros::Publisher>& odomPublishers) {
    for (size_t i = 0; i < robotList.size(); ++i) {
        posePublishers.push_back(nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + robotList[i].name + "_pose", 10));
        scanPublishers.push_back(nodeHandle.advertise<sensor_msgs::LaserScan>("/" + robotList[i].name + "_scan", 10));
        odomPublishers.push_back(nodeHandle.advertise<nav_msgs::Odometry>("/" + robotList[i].name + "_odom", 10));
    }
    nodeHandle.subscribe("/cmd_vel", 10, velocityCallback);
    nodeHandle.subscribe("/move_base_simple/goal", 10, goalCallback);
}

// Update robot positions and publish their states
void updateRobots(double deltaTime, std::vector<ros::Publisher>& posePublishers, 
                  std::vector<ros::Publisher>& odomPublishers, tf::TransformBroadcaster& transformBroadcaster) {
    for (size_t i = 0; i < robotList.size(); ++i) {
        if (selectedRobot == robotList[i].name) {
            robotVelocities[i] = robotVelocity;
        }

        // Update robot pose based on velocity
        robotList[i].pose.theta += robotVelocities[i].angular.z * deltaTime;
        robotList[i].pose.x += robotVelocities[i].linear.x * cos(robotList[i].pose.theta) * deltaTime;
        robotList[i].pose.y += robotVelocities[i].linear.x * sin(robotList[i].pose.theta) * deltaTime;

        // Normalize angle
        if (robotList[i].pose.theta >= 2.0 * M_PI) {
            robotList[i].pose.theta -= 2.0 * M_PI;
        } else if (robotList[i].pose.theta < 0.0) {
            robotList[i].pose.theta += 2.0 * M_PI;
        }

        // Publish pose and odometry data
        geometry_msgs::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.header.frame_id = robotList[i].frame_id;
        poseMsg.pose.pose.position.x = robotList[i].pose.x;
        poseMsg.pose.pose.position.y = robotList[i].pose.y;
        poseMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotList[i].pose.theta);
        posePublishers[i].publish(poseMsg);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time::now();
        odometry.header.frame_id = "map";
        odometry.pose.pose.position.x = robotList[i].pose.x;
        odometry.pose.pose.position.y = robotList[i].pose.y;
        odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotList[i].pose.theta);
        odometry.child_frame_id = robotList[i].name + "_baselink";
        odometry.twist.twist = robotVelocities[i];
        odomPublishers[i].publish(odometry);

        // Broadcast transform for robot base link
        geometry_msgs::TransformStamped transformMsg;
        transformMsg.header.frame_id = robotList[i].frame_id;
        transformMsg.transform.translation.x = robotList[i].pose.x;
        transformMsg.transform.translation.y = robotList[i].pose.y;
        transformMsg.transform.rotation = tf::createQuaternionMsgFromYaw(robotList[i].pose.theta);
        transformMsg.header.stamp = ros::Time::now();
        transformMsg.child_frame_id = robotList[i].name + "_baselink";
        transformBroadcaster.sendTransform(transformMsg);

        // Broadcast transform for laser scan frame
        geometry_msgs::TransformStamped sensorTransform;
        sensorTransform.header.frame_id = robotList[i].name + "_baselink";
        sensorTransform.transform.rotation = tf::createQuaternionMsgFromYaw(-robotList[i].pose.theta);
        sensorTransform.header.stamp = ros::Time::now();
        sensorTransform.child_frame_id = robotList[i].name + "_scan_frame";
        transformBroadcaster.sendTransform(sensorTransform);

        // Broadcast odom to base link transform
        geometry_msgs::TransformStamped odomTransform;
        odomTransform.header.frame_id = "odom";
        odomTransform.transform.translation.x = robotList[i].pose.x;
        odomTransform.transform.translation.y = robotList[i].pose.y;
        odomTransform.transform.rotation = tf::createQuaternionMsgFromYaw(robotList[i].pose.theta);
        odomTransform.header.stamp = ros::Time::now();
        odomTransform.child_frame_id = robotList[i].name + "_baselink";
        transformBroadcaster.sendTransform(odomTransform);
    }
}

// Main simulation loop
void executeSimulation(ros::NodeHandle& nodeHandle, ros::Rate& loopRate, std::vector<ros::Publisher>& posePublishers, 
                       std::vector<ros::Publisher>& scanPublishers, std::vector<ros::Publisher>& odomPublishers, 
                       tf::TransformBroadcaster& transformBroadcaster) {
    ros::Time currentTime, lastTime;
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    while (ros::ok()) {
        // Prepare visualization map
        visualizationMap = originalMap.clone();
        cv::cvtColor(visualizationMap, grayscaleMap, cv::COLOR_BGR2GRAY);

        visualizationMap = add_robots(visualizationMap, mapConfig, robotList);
        visualizationMap = Lidar(visualizationMap, grayscaleMap, mapConfig, robotList, grayThreshold, scanPublishers);

        cv::imshow("multi_xy_robot_simulator", visualizationMap);
        int key = cv::waitKey(1);

        // Broadcast map to odom transform
        geometry_msgs::TransformStamped mapTransform;
        mapTransform.header.frame_id = "map";
        mapTransform.transform.rotation = tf::createQuaternionMsgFromYaw(0);
        mapTransform.header.stamp = ros::Time::now();
        mapTransform.child_frame_id = "odom";
        transformBroadcaster.sendTransform(mapTransform);

        currentTime = ros::Time::now();
        double deltaTime = (currentTime - lastTime).toSec();
        updateRobots(deltaTime, posePublishers, odomPublishers, transformBroadcaster);

        lastTime = currentTime;
        ros::spinOnce();
        loopRate.sleep();
    }
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "xy_simulator");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(10);

    // Load configuration and initialize robots
    yamlConfig = YAML::LoadFile("/home/raja/catkin_ws/src/rp/config/config.yaml");
    mapConfig = extract_map(yamlConfig);
    robotList = extractRobots(yamlConfig);

    draggingRobots.resize(robotList.size(), false);
    robotVelocities.resize(robotList.size());

    if (argc != 2) {
        std::cerr << "Usage: " << "rosrun xy_simulator main" << " <robot_name>" << std::endl;
        selectedRobot = "Bot1";
    } else {
        selectedRobot = argv[1];
    }

    std::cout << "Selected robot for navigation stack: " << selectedRobot << std::endl;

    // Load and set up the map
    originalMap = cv::imread(mapConfig.path);
    mapConfig.width = originalMap.cols * mapConfig.resolution;
    mapConfig.height = originalMap.rows * mapConfig.resolution;
    std::cout << "rows: " << originalMap.rows << " cols: " << originalMap.cols << std::endl;
    cv::namedWindow("multi_xy_robot_simulator");
    cv::setMouseCallback("multi_xy_robot_simulator", mouseHandler);

    std::vector<ros::Publisher> posePublishers;
    std::vector<ros::Publisher> scanPublishers;
    std::vector<ros::Publisher> odomPublishers;
    tf::TransformBroadcaster transformBroadcaster;

    // Initialize ROS communication
    setupROS(nodeHandle, posePublishers, scanPublishers, odomPublishers);

    // Run the simulation
    executeSimulation(nodeHandle, loopRate, posePublishers, scanPublishers, odomPublishers, transformBroadcaster);

    return 0;
}

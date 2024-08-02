#ifndef ROBOTS_H
#define ROBOTS_H

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <iostream>

// Pose structure to store the robot's position and orientation.
struct Pose 
{
    double x;
    double y;
    double theta;
};

// Structure to store the robot's dimensions.
struct Dimensions 
{
    double radius;
};

// Structure to store the robot's color.
struct Color 
{
    double r;
    double g;
    double b;
    double a;
};

// Structure to store the robot's LiDAR configuration.
struct Lidar 
{
    int num_beams;
    double min_range;
    double max_range;
};

// Structure representing a robot with its properties.
struct Robot 
{
    std::string name;
    std::string frame_id;
    Pose pose;
    Dimensions dimensions;
    Color color;
    Lidar lidar;
    double max_vel;
};

// Function declaration for extracting robot configuration.
std::vector<Robot> extractRobots(const YAML::Node& config);

#endif // ROBOTS_H

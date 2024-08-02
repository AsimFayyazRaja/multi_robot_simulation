#include "visualize.h"
#include <iostream>

// Converts world coordinates to image coordinates.
cv::Point2d worldToImage(const Pose& pose, const MapConfig& mapConfig) 
{
    int x = static_cast<int>((pose.x + mapConfig.width / 2) / mapConfig.resolution);
    int y = static_cast<int>((-pose.y + mapConfig.height / 2) / mapConfig.resolution);

    return cv::Point2d(x, y);
}

// Converts image coordinates to world coordinates.
Pose imageToWorld(const cv::Point2d& image_point, const MapConfig& mapConfig) 
{
    double x = (image_point.x * mapConfig.resolution) - mapConfig.width / 2;
    double y = -(image_point.y * mapConfig.resolution) + mapConfig.height / 2;
    
    return {x, y, 0}; 
}

// Draws an arrow on the map representing the robot's orientation.
cv::Mat drawArrow(cv::Mat& image, const cv::Point2i& start, double angle, const cv::Scalar& color, double size) 
{ 
    cv::Point2i end(start.x + size * std::cos(-angle), start.y + size * std::sin(-angle));
    cv::arrowedLine(image, start, end, color, 2);
    return image;  
}

// Draws the robots on the map.
cv::Mat add_robots(cv::Mat& mapImage, const MapConfig& mapConfig, const std::vector<Robot>& robots)
{
    for (const auto& robot : robots) 
    {
        cv::Scalar color_a(0, 0, 0, 255); // Arrow color.
        cv::Scalar color_r(robot.color.b * 255, robot.color.g * 255, robot.color.r * 255, robot.color.a * 128); // Robot color.

        int radius = robot.dimensions.radius / mapConfig.resolution; // Convert radius to map scale.
        cv::Point2d center = worldToImage(robot.pose, mapConfig);

        cv::circle(mapImage, center, radius, color_r, cv::FILLED); // Draw robot.
        cv::circle(mapImage, center, radius, cv::Scalar(0, 0, 0, 255), 1); // Draw robot boundary.
        mapImage = drawArrow(mapImage, center, robot.pose.theta, color_a, radius - 2); // Draw orientation arrow.
    }

    return mapImage;         
}

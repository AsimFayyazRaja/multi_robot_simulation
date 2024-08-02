#ifndef LIDAR_H
#define LIDAR_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "robots.h"
#include "map.h"
#include "visualize.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// Function declaration for LiDAR simulation.
cv::Mat Lidar(cv::Mat& mapImage, cv::Mat& gray_image, const MapConfig& mapConfig,  
              std::vector<Robot>& robots, const int intensity_threshold, std::vector<ros::Publisher> lidar_publisher);

#endif //LIDAR_H

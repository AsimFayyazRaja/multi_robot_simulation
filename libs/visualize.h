#ifndef visualize_H
#define visualize_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "robots.h"
#include "map.h"
#include <iterator>

// Function declarations for visualization utilities.
cv::Point2d worldToImage(const Pose& pose, const MapConfig& mapConfig);

Pose imageToWorld(const cv::Point2d& image_point, const MapConfig& mapConfig);

cv::Mat drawArrow(cv::Mat& image, const cv::Point2i& start, double angle, const cv::Scalar& color, double size);

cv::Mat add_robots(cv::Mat& mapImage, const MapConfig& mapConfig, const std::vector<Robot>& robots);

#endif // visualize_H

#include "lidar.h"

// Function to simulate LiDAR scanning of the environment.
cv::Mat Lidar(cv::Mat& mapImage, cv::Mat& gray_image, const MapConfig& mapConfig,  
              std::vector<Robot>& robots, const int intensity_threshold, std::vector<ros::Publisher> lidar_publishers)
{
    std::vector<sensor_msgs::LaserScan> laserScans; // Container to store laser scan data for each robot.

    // Iterate over each robot in the simulation.
    for (size_t r = 0; r < robots.size(); ++r)
    {
        Robot robot = robots[r];
        int num_beams = robot.lidar.num_beams; // Number of LiDAR beams
        double min_range = robot.lidar.min_range + robot.dimensions.radius + 1; // Minimum LiDAR range
        double max_range = robot.lidar.max_range; // Maximum LiDAR range
        cv::Scalar color(robot.color.b * 255, robot.color.g * 255, robot.color.r * 255, robot.color.a * 128); // LiDAR beam color

        // Create a copy of robots excluding the current robot.
        std::vector<Robot> robotsCopy = robots;
        robotsCopy.erase(std::remove_if(robotsCopy.begin(), robotsCopy.end(), [&robot](const Robot& robot1) { return robot1.name == robot.name; }), robotsCopy.end());

        // Draw other robots on the map to simulate the environment.
        gray_image = add_robots(gray_image, mapConfig, robotsCopy);
        
        sensor_msgs::LaserScan lidar_data; // Initialize LiDAR data message.
        lidar_data.header.stamp = ros::Time::now();
        lidar_data.header.frame_id = robot.name + "_scan_frame";  
        lidar_data.angle_min = 0;  
        lidar_data.angle_max = 2 * M_PI; 
        lidar_data.angle_increment = (2 * M_PI) / num_beams;
        lidar_data.time_increment = 0.1 / num_beams;   
        lidar_data.scan_time = 0.1;       
        lidar_data.range_min = min_range;     
        lidar_data.range_max = max_range;
        lidar_data.ranges = std::vector<float>(num_beams, 0.0);

        // Simulate the LiDAR scan.
        for (int i = 0; i < num_beams; ++i) 
        {
            double angle = i * (2.0 * M_PI / num_beams); // Angle in radians

            // Check each range along the beam.
            for (double range = min_range; range <= max_range; range += 0.01) 
            {
                // Convert polar coordinates to Cartesian coordinates
                cv::Point2d map = worldToImage({robot.pose.x + range * cos(angle), robot.pose.y + range * sin(angle), 0}, mapConfig);

                // Check if the point is inside the 2D map bounds
                if (map.x >= 0 && map.x < gray_image.cols && map.y >= 0 && map.y < gray_image.rows) 
                {
                    // Check if the intensity of the map at the point is below the threshold, indicating a hit.
                    if (gray_image.at<uchar>(map.y, map.x) < intensity_threshold) 
                    {
                        lidar_data.ranges[i] = range;
                        cv::circle(mapImage, cv::Point(map.x, map.y), 2, color, -1); // Draw the detected point.
                        break; // Break after detecting a boundary point.
                    }
                }
            }
        }
        
        // Publish the LiDAR scan data for the current robot.
        lidar_publishers[r].publish(lidar_data);
    }
    return mapImage;
}

#include "robots.h"

// Extracts robot configuration from a YAML node.
std::vector<Robot> extractRobots(const YAML::Node& config) 
{
    std::vector<Robot> robots;

    if (config["robots"])
    {
        for (const auto& robotNode : config["robots"]) 
        {
            Robot robot;
            robot.name = robotNode["name"].as<std::string>();
            std::cout << robot.name << std::endl;
            robot.frame_id = robotNode["frame_id"].as<std::string>();
            robot.pose.x = robotNode["initial_pose"]["x"].as<double>();
            robot.pose.y = robotNode["initial_pose"]["y"].as<double>();
            robot.pose.theta = robotNode["initial_pose"]["theta"].as<double>();
            robot.dimensions.radius = robotNode["dimensions"]["wheel_r"].as<double>();
            robot.color.r = robotNode["color"]["r"].as<double>();
            robot.color.g = robotNode["color"]["g"].as<double>();
            robot.color.b = robotNode["color"]["b"].as<double>();
            robot.color.a = robotNode["color"]["a"].as<double>();
            robot.lidar.num_beams = robotNode["lidar"]["num_beams"].as<int>();
            robot.lidar.min_range = robotNode["lidar"]["min_range"].as<double>();
            robot.lidar.max_range = robotNode["lidar"]["max_range"].as<double>();
            robot.max_vel = robotNode["max_vel"].as<double>();

            robots.push_back(robot);
        }
    }
    else
    {
        // Print debug information
        std::cout << "YAML Exception while extracting Robots: " << std::endl;
        // Rethrow the exception to terminate the program
        throw;
    }

    return robots;
}

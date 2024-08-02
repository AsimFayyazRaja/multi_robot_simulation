#include "map.h"

// Extracts map configuration from a YAML node.
MapConfig extract_map(const YAML::Node& config) 
{
    MapConfig mapconfig;

    if (config["map"])
    { 
        mapconfig.path = config["map"]["path"].as<std::string>();
        mapconfig.width = config["map"]["width"].as<double>();           
        mapconfig.height = config["map"]["height"].as<double>();
        mapconfig.resolution = config["map"]["resolution"].as<double>();
    }
    else 
    {
        // Print debug information
        std::cout << "YAML Exception while extracting map: " << std::endl;
        // Terminate the program
        throw;
    }

    return mapconfig;
}

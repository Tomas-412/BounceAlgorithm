#ifndef POINT_CLOUD_PROCESSOR_HPP_
#define POINT_CLOUD_PROCESSOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>

class PointCloudProcessor
{
public:
    PointCloudProcessor() = default;

    // Function to save cloud points in a file
    void saveCloudPointInFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const rclcpp::Logger &logger)
    {
        std::ofstream output_file("lidar_data/point_cloud_data.txt");

        if (output_file.is_open())
        {
            output_file << "X, Y, Z\n"; // Write the header for the file

            // Loop over all points in the cloud
            for (size_t i = 0; i < cloud->size(); ++i)
            {
                pcl::PointXYZ point = cloud->points[i];
                output_file << point.x << ", " << point.y << ", " << point.z << "\n"; // Write X, Y, Z to the file
            }

            output_file.close(); // Close the file
            RCLCPP_INFO(logger, "Saved %zu points to file.", cloud->size());
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to open file for writing.");
        }
    }

    // Function to get the nearest point and its distance
    std::pair<pcl::PointXYZ, float> getNearestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::PointXYZ nearest_point;
        float min_distance = std::numeric_limits<float>::max();

        // Iterate through each point in the cloud
        for (const auto &point : cloud->points)
        {
            // Calculate the Euclidean distance from the origin (0, 0, 0)
            float distance = std::sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));

            // If this point is closer, store it as the nearest point
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_point = point;
            }
        }

        // Return both the nearest point and its distance
        return std::make_pair(nearest_point, min_distance);
    }

    // Function to filter ground points based on Z coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float lidar_height, float ground_threshold = 0.1)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Loop through the input point cloud
        for (const auto &point : input_cloud->points)
        {
            // Check if the point is above the ground level
            if (point.z > -lidar_height + ground_threshold)
            {
                filtered_cloud->points.push_back(point); // If above the ground, keep the point
            }
        }

        // Set the filtered cloud's width and height
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1; // Unstructured point cloud
        filtered_cloud->is_dense = true;

        return filtered_cloud;
    }
};

#endif // POINT_CLOUD_PROCESSOR_HPP_

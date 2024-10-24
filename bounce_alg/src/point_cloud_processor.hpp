#ifndef POINT_CLOUD_PROCESSOR_HPP_
#define POINT_CLOUD_PROCESSOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <vector>
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundHPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float lidar_height, float ground_threshold = 0.05)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Loop through the input point cloud
        for (const auto &point : input_cloud->points)
        {
            // Check if the point is above the ground level
            if (point.z > -lidar_height + ground_threshold || point.z < -lidar_height - ground_threshold)
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

    // Function to filter ground points based on 3 point angle and plane fitting
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        // Create the GroundPlaneFilter object with desired parameters
        tolerance = 0.1f;         // Dot product tolerance
        ground_threshold = 0.04f; // Distance threshold for ground filtering

        // Step 1: Estimate initial ground points using dot product analysis
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points = estimateGroundPoints(input_cloud);

        // Step 2: Fit ground plane using RANSAC
        pcl::ModelCoefficients::Ptr plane_coefficients = fitGroundPlane(ground_points);

        // Step 3: Filter ground points based on distance to the plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterGroundPoints(input_cloud, plane_coefficients);

        return filtered_cloud;
    }

private:
    // Helper function to compute dot product and check if points form a ground plane
    bool isGroundPoint(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2, const pcl::PointXYZ &point3)
    {
        Eigen::Vector3f vector1(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
        Eigen::Vector3f vector2(point3.x - point2.x, point3.y - point2.y, point3.z - point2.z);

        // Compute dot product
        float dotProduct = vector1.dot(vector2) / (vector1.norm() * vector2.norm());

        // Check if the dot product is close to 1 (indicating points lie on a flat surface)
        return std::fabs(dotProduct - 1.0) < tolerance;
    }

    // Step 1: Estimate ground points using dot product
    pcl::PointCloud<pcl::PointXYZ>::Ptr estimateGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < input_cloud->points.size() - 2; i++)
        {
            if (isGroundPoint(input_cloud->points[i], input_cloud->points[i + 1], input_cloud->points[i + 2]))
            {
                ground_points->points.push_back(input_cloud->points[i]);
                ground_points->points.push_back(input_cloud->points[i + 1]);
                ground_points->points.push_back(input_cloud->points[i + 2]);
            }
        }

        return ground_points;
    }

    // Step 2: Fit ground plane using RANSAC
    pcl::ModelCoefficients::Ptr fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01); // Adjust based on precision needed

        seg.setInputCloud(ground_cloud);
        seg.segment(*inliers, *coefficients);

        return coefficients; // The plane model coefficients (a, b, c, d) for the plane equation
    }

    // Helper function to compute distance from point to plane
    float pointToPlaneDistance(const pcl::PointXYZ &point, const pcl::ModelCoefficients::Ptr &coefficients)
    {
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];

        // Compute the distance from the point to the plane
        float distance = std::fabs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(pow(a,2) + pow(b,2) + pow(c,2));
        return distance;
    }

    // Step 3: Filter ground points based on distance to the fitted plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const pcl::ModelCoefficients::Ptr &coefficients)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &point : input_cloud->points)
        {
            float distance = pointToPlaneDistance(point, coefficients);

            // Check if the point is above the threshold distance from the plane
            if (distance > ground_threshold)
            {
                filtered_cloud->points.push_back(point); // Keep non-ground points
            }
        }

        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1; // Unstructured point cloud
        filtered_cloud->is_dense = true;

        return filtered_cloud;
    }

    float tolerance;
    float ground_threshold;
};

#endif // POINT_CLOUD_PROCESSOR_HPP_

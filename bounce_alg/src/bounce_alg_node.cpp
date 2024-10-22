#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream> // for file operations
#include <limits>  // for std::numeric_limits
#include <cmath>   // for std::sqrt
#include <utility> // for std::pair

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp" // Message type for velocity command
#include <geometry_msgs/msg/point_stamped.hpp>

//Custom classes
#include "velocity_controller.hpp"
#include "point_cloud_processor.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BounceAlgorithmNode : public rclcpp::Node
{
public:
  BounceAlgorithmNode() : Node("bounce_alg")
  {
    // Initialize the PointCloudProcessor
    point_cloud_processor_ = std::make_shared<PointCloudProcessor>();


    // Initialize the VelocityController with relevant thresholds and gains
    velocity_controller_ = std::make_shared<VelocityController>(
        1, 1.5, 0.9);
        //stop_thresh, slow_thresh, obstacle_zone_y

    // Declare parameters with default values
    this->declare_parameter<double>("linear_vel", 1.5);
    this->declare_parameter<double>("angular_vel", 1);

    // Create a subscriber on the /a200_0000/sensors/lidar3d_0/points topic
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/a200_0000/sensors/lidar3d_0/points", 10,
        std::bind(&BounceAlgorithmNode::pointCloudCallback, this, std::placeholders::_1));

    // Initialize the publisher on the /a200_0000/cmd_vel topic, sending geometry_msgs/Twist messages
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/a200_0000/cmd_vel", 10);

    // Initialize the publisher (in constructor)
    filtered_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

    // Initialize the publisher (in constructor or init method)
    nearest_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("nearest_point", 10);

    // Set the timer to call the publish_velocity function at a rate of 20 Hz (50ms interval)
    timer_ = this->create_wall_timer(50ms, std::bind(&BounceAlgorithmNode::publishVelocity, this));
  }

private:
  void publishVelocity()
  {

    // Process the point cloud and find the nearest point (filtered cloud without ground)
    // Assuming nearestPoint and nearestPointDistance have been calculated

    double linear_vel_param, angular_vel_param;
    this->get_parameter("linear_vel", linear_vel_param);
    this->get_parameter("angular_vel", angular_vel_param);

    // Use the VelocityController to compute the velocity command
    geometry_msgs::msg::Twist message = velocity_controller_->calculateVelocity(
      nearest_point_distance_, nearest_point_, linear_vel_param, angular_vel_param);


    RCLCPP_INFO(this->get_logger(), "Publishing velocity: linear.x = %f, angular.z = %f", message.linear.x, message.angular.z);
    velocity_pub_->publish(message); // Publish the velocity command
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Save Cloud point in a file if needed
    // point_cloud_processor_->saveCloudPointInFile(cloud);

    // Filter out the ground plane
    float lidar_height = 0.28; // Example height of the LiDAR (in meters)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = point_cloud_processor_->filterGroundPlane(cloud, lidar_height);

    // Convert the filtered cloud to a ROS message
    sensor_msgs::msg::PointCloud2 outputCloud_msg;
    pcl::toROSMsg(*filtered_cloud, outputCloud_msg);
    outputCloud_msg.header = msg->header; // Maintain the same header for time and frame

    // Publish the filtered point cloud
    filtered_cloud_pub->publish(outputCloud_msg);

    // debug part
    // RCLCPP_INFO(this->get_logger(), "Filtered point cloud with %zu points", filtered_cloud->size());

    // Get the nearest point and its distance
    auto nearest_point_and_distance = point_cloud_processor_->getNearestPoint(filtered_cloud);
    nearest_point_ = nearest_point_and_distance.first;
    nearest_point_distance_ = nearest_point_and_distance.second;

    // Create a PointStamped message for the nearest point
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = msg->header; // Maintain the same header for time and frame

    // Set the coordinates of the point
    point_msg.point.x = nearest_point_.x;
    point_msg.point.y = nearest_point_.y;
    point_msg.point.z = nearest_point_.z;

    // Publish the point
    nearest_point_pub->publish(point_msg);
  }

  /* void saveCloudPointInFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // Open a text file to save the cloud points
    // std::ofstream output_file("/home/tomas-jelinek/obstacle_avoidance/cloudpoint_example/point_cloud_data.txt");

    std::ofstream output_file("lidar_data/point_cloud_data.txt");

    if (output_file.is_open())
    {
      output_file << "X, Y, Z\n"; // Write the header for the file

      // Loop over the first few points (for example, the first 10)
      for (size_t i = 0; i < cloud->size(); ++i)
      {
        pcl::PointXYZ point = cloud->points[i];
        output_file << point.x << ", " << point.y << ", " << point.z << "\n"; // Write X, Y, Z to the file
      }

      output_file.close(); // Close the file
      RCLCPP_INFO(this->get_logger(), "Saved %zu points to file.", cloud->size());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
    }
  }

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

  // Function to filter out ground points based on Z coordinate
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float lidar_height, float ground_threshold = 0.1)
  {
    // Create a new point cloud to hold the filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Loop through the input point cloud
    for (const auto &point : input_cloud->points)
    {
      // Check if the point is above the ground level
      if (point.z > -lidar_height + ground_threshold)
      {
        // If above the ground, keep the point
        filtered_cloud->points.push_back(point);
      }
    }

    // Set the filtered cloud's width and height
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1; // Unstructured point cloud
    filtered_cloud->is_dense = true;

    return filtered_cloud;
  } */

  // Custom classes
  std::shared_ptr<VelocityController> velocity_controller_; // Instance of the new VelocityController class
  std::shared_ptr<PointCloudProcessor> point_cloud_processor_; // Instance of the new PointCloudProcessor class

  // Shared variables
  float nearest_point_distance_;
  pcl::PointXYZ nearest_point_;

  // Subscriber for LiDAR 3D cloud points
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  // Publishers for velocity, filtered point cloud and nearest point
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_point_pub;

  // Timer to have a desired publish frequency
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                              // Initialize ROS 2
  rclcpp::spin(std::make_shared<BounceAlgorithmNode>()); // Spin the node to keep it alive
  rclcpp::shutdown();                                    // Shutdown ROS 2
  return 0;
}
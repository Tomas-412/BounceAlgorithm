#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream> // for file operations
#include <limits> // for std::numeric_limits
#include <cmath>  // for std::sqrt
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



using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BounceAlgorithmNode : public rclcpp::Node
{
public:
  BounceAlgorithmNode() : Node("bounce_alg")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("linear_vel", 1);
    this->declare_parameter<double>("angular_vel", 0);

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
    // By default the velocity is 0
    double linear_vel = 0;
    double angular_vel = 0;

    // Debug, if the shared variables are working
    RCLCPP_INFO(this->get_logger(), "DISTANCE = %f",nearestPointDistance);

    // If the nearest object is closer than 1 meter and it is in front of the robot and max 1.5 meter to the sides
    if(nearestPointDistance < 1 and nearestPoint.x > 0 and abs(nearestPoint.y) < 1.5)
    {
      // The robot will stop and turn to the right if the obstacle is on the left side and vice versa
      if(nearestPoint.y > 0) angular_vel = -0.5;
      else angular_vel = 0.5;
    }
    else // If the object is further than one meter the robot goes based on the parameters
    {
      this->get_parameter("linear_vel", linear_vel);
      this->get_parameter("angular_vel", angular_vel);
    }

    auto message = geometry_msgs::msg::Twist();
    // Set the linear and angular velocity based on the parameters 
    message.linear.x = linear_vel;   
    message.angular.z = angular_vel;


    RCLCPP_INFO(this->get_logger(), "Publishing velocity: linear.x = %f, angular.z = %f", message.linear.x, message.angular.z);
    velocity_pub_->publish(message); // Publish the velocity command
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // debug part
    //RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->size());

    // Save Cloud point in a file if needed
    // saveCloudPointInFile(cloud);

    // Filter out the ground plane
    float lidar_height = 0.28; // Example height of the LiDAR (in meters)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterGroundPlane(cloud, lidar_height);

    // Convert the filtered cloud to a ROS message
    sensor_msgs::msg::PointCloud2 outputCloud_msg;
    pcl::toROSMsg(*filtered_cloud, outputCloud_msg);
    outputCloud_msg.header = msg->header; // Maintain the same header for time and frame

    // Publish the filtered point cloud
    filtered_cloud_pub->publish(outputCloud_msg);

    // debug part
    //RCLCPP_INFO(this->get_logger(), "Filtered point cloud with %zu points", filtered_cloud->size());

    // Get the nearest point and its distance
    auto nearest_point_and_distance = getNearestPoint(filtered_cloud);
    nearestPoint = nearest_point_and_distance.first;
    nearestPointDistance = nearest_point_and_distance.second;

    // Print the nearest point's coordinates and the distance
    RCLCPP_INFO(this->get_logger(), "Nearest point: X = %f, Y = %f, Z = %f, Distance = %f", 
                nearestPoint.x, nearestPoint.y, nearestPoint.z, nearestPointDistance);


    // Create a PointStamped message for the nearest point
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = msg->header; // Maintain the same header for time and frame
    
    // Set the coordinates of the point
    point_msg.point.x = nearestPoint.x;
    point_msg.point.y = nearestPoint.y;
    point_msg.point.z = nearestPoint.z;

    // Publish the point
    nearest_point_pub->publish(point_msg);
  }

  void saveCloudPointInFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
    for (const auto& point : cloud->points)
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
  }

  // Shared variables 
  float nearestPointDistance; 
  pcl::PointXYZ nearestPoint;

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
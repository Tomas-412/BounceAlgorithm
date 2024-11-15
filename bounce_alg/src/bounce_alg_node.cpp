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

// Custom classes
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
    // ---------------------------------------------------------------------------------------------------------------
    // Declare parameters with default values
    // ---------------------------------------------------------------------------------------------------------------

    this->declare_parameter<double>("linear_vel"); // cmd_vel robot cotrol
    this->declare_parameter<double>("angular_vel");

    this->declare_parameter<float>("slow_threshold"); // obstacle avoidance parameters
    this->declare_parameter<float>("stop_threshold");
    this->declare_parameter<float>("obstacle_zone_y");


    this->declare_parameter<float>("ground_tolerance"); // filter ground plane parameters
    this->declare_parameter<float>("ground_treshold");


    this->declare_parameter<bool>("simulation"); // switch easily between simulated and real environment

    // ---------------------------------------------------------------------------------------------------------------
    // Initialize the PointCloudProcessor
    // ---------------------------------------------------------------------------------------------------------------#

    auto slow_thresh = 0.0;
    auto stop_thresh = 0.0;
    auto obst_zone_y = 0.0;
    this->get_parameter("slow_threshold", slow_thresh);
    this->get_parameter("stop_threshold", stop_thresh);
    this->get_parameter("obstacle_zone_y", obst_zone_y);


    point_cloud_processor_ = std::make_shared<PointCloudProcessor>();

    // Initialize the VelocityController with relevant thresholds and gains
    velocity_controller_ = std::make_shared<VelocityController>(
        stop_thresh, slow_thresh, obst_zone_y);
    // stop_thresh, slow_thresh, obstacle_zone_y



    // ---------------------------------------------------------------------------------------------------------------
    // Set topics accordingly based on if it is simulation
    // ---------------------------------------------------------------------------------------------------------------

    auto LidarTopic = "";
    auto VelocityTopic = "";
    bool simulation;
    this->get_parameter("simulation", simulation);

    if (simulation)
    {
      LidarTopic = "/a200_0000/sensors/lidar3d_0/points";
      VelocityTopic = "/a200_0000/cmd_vel";
    }
    else
    {
      LidarTopic = "/velodyne_points";
      VelocityTopic = "/cmd_vel";
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Create all the needed subscribers
    // ---------------------------------------------------------------------------------------------------------------

    // Create a subscriber on the /a200_0000/sensors/lidar3d_0/points topic
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        LidarTopic, 10,
        std::bind(&BounceAlgorithmNode::pointCloudCallback, this, std::placeholders::_1));

    // ---------------------------------------------------------------------------------------------------------------
    // Create all the needed publishers
    // ---------------------------------------------------------------------------------------------------------------

    point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("first_16_points_cloud", 10);

    // Initialize the publisher on the /a200_0000/cmd_vel topic, sending geometry_msgs/Twist messages
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(VelocityTopic, 10);

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

    // Check if the cloud has valid points when received
    if (cloud->width == 0 || cloud->height == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Received empty point cloud.");
      return;
    }

    // Save Cloud point in a file if needed
    // point_cloud_processor_->saveCloudPointInFile(cloud);

    // Filter out the ground plane
    float tolerance, treshold;
    this->get_parameter("ground_tolerance", tolerance);
    this->get_parameter("ground_treshold", treshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = point_cloud_processor_->filterGroundPlane(cloud, tolerance, treshold);

    // Filter points above the robot so it can pass under an obstacle (default 20 cm)
    filtered_cloud = point_cloud_processor_->filterPointsAbove(filtered_cloud, 0.2);

    // Convert the filtered cloud to a ROS message
    sensor_msgs::msg::PointCloud2 outputCloud_msg;
    pcl::toROSMsg(*filtered_cloud, outputCloud_msg);
    outputCloud_msg.header = msg->header; // Maintain the same header for time and frame

    // Publish the filtered point cloud
    filtered_cloud_pub->publish(outputCloud_msg);

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

    // Get the first 16 points from the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr first_16_points = point_cloud_processor_->getFirst16Points(cloud);

    // Visualize the first 16 points in RViz
    publishFirst16Points(first_16_points, msg->header);
  }

  void publishFirst16Points(const pcl::PointCloud<pcl::PointXYZ>::Ptr &first_16_points, const std_msgs::msg::Header &header)
  {
    // Create a PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*first_16_points, output_cloud);  // Convert the PCL point cloud to a ROS message
    output_cloud.header = header;  // Use the same header for consistency

    // Publish the point cloud
    point_cloud_pub->publish(output_cloud);
  }

  // Custom classes
  std::shared_ptr<VelocityController> velocity_controller_;    // Instance of the new VelocityController class
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

  // Publisher for markers in RViz
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;

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
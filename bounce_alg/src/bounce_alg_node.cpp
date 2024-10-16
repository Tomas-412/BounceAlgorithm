#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream> // for file operations

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp" // Message type for velocity command

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BounceAlgorithmNode : public rclcpp::Node
{
public:
  BounceAlgorithmNode() : Node("bounce_alg")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("linear_vel", 0.5);
    this->declare_parameter<double>("angular_vel", 0);

    // Create a subscriber on the /a200_0000/sensors/lidar3d_0/points topic
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/a200_0000/sensors/lidar3d_0/points", 10,
        std::bind(&BounceAlgorithmNode::pointCloudCallback, this, std::placeholders::_1));

    // Create a publisher on the /a200_0000/cmd_vel topic, sending geometry_msgs/Twist messages
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/a200_0000/cmd_vel", 11);

    // Set the timer to call the publish_velocity function at a rate of 20 Hz (50ms interval)
    timer_ = this->create_wall_timer(50ms, std::bind(&BounceAlgorithmNode::publishVelocity, this));
  }

private:
  void publishVelocity()
  {
    // Get parameter values
    double linear_vel;
    double angular_vel;

    this->get_parameter("linear_vel", linear_vel);
    this->get_parameter("angular_vel", angular_vel);

    auto message = geometry_msgs::msg::Twist();
    message.linear.x = linear_vel;   // Set the linear velocity to go straight forward (1.0 m/s)
    message.angular.z = angular_vel; // No angular rotation (go straight)

    RCLCPP_INFO(this->get_logger(), "Publishing velocity: linear.x = %f, angular.z = %f", message.linear.x, message.angular.z);
    velocity_pub_->publish(message); // Publish the velocity command
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // debug part
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->size());

    // Open a text file to save the cloud points
    // std::ofstream output_file("/home/tomas-jelinek/obstacle_avoidance/cloudpoint_example/point_cloud_data.txt");

    std::ofstream output_file("point_cloud_data.txt");

    if (output_file.is_open())
    {
      output_file << "X, Y, Z\n"; // Write the header for the file

      // Loop over the first few points (for example, the first 10)
      for (size_t i = 0; i < msg->width * msg->height; ++i)
      {
        pcl::PointXYZ point = cloud->points[i];
        output_file << point.x << ", " << point.y << ", " << point.z << "\n"; // Write X, Y, Z to the file
      }

      output_file.close(); // Close the file
      RCLCPP_INFO(this->get_logger(), "Saved %zu points to file.", std::min(cloud->size(), msg->width * msg->height));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                              // Initialize ROS 2
  rclcpp::spin(std::make_shared<BounceAlgorithmNode>()); // Spin the node to keep it alive
  rclcpp::shutdown();                                    // Shutdown ROS 2
  return 0;
}
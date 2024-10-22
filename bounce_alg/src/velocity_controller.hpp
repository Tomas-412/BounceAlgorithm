#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include <pcl/point_types.h>

class VelocityController
{
public:
    VelocityController(
        double stop_thresh, double slow_thresh, double obstacle_zone_y)
        : stop_threshold_(stop_thresh), slow_threshold_(slow_thresh),
          obstacle_zone_y_(obstacle_zone_y) {}

    // Function to calculate the velocity command
    geometry_msgs::msg::Twist calculateVelocity(double nearestPointDistance, const pcl::PointXYZ &nearestPoint, double linear_vel_param, double angular_vel_param)
    {

        geometry_msgs::msg::Twist cmd_vel;

        double linear_vel = 0.0;
        double angular_vel = 0.0;

        if (nearestPointDistance < stop_threshold_ && nearestPoint.x > 0 && abs(nearestPoint.y) < obstacle_zone_y_)
        {
            // Stop and turn
            angular_vel = (nearestPoint.y > 0) ? -angular_vel_param : angular_vel_param; // Turn right if obstacle on the left, and vice versa
        }
        else if (nearestPointDistance < slow_threshold_)
        {
            // Slow down and adjust direction
            linear_vel = linear_vel_param * (nearestPointDistance / slow_threshold_);
            angular_vel = -0.2 * (obstacle_zone_y_ / nearestPoint.y); // Proportional steering
        }
        else
        {
            // No critical obstacle, proceed with default parameters
            linear_vel = linear_vel_param;
        }

        // Set velocities in the Twist message
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        return cmd_vel;
    }

private:
    double stop_threshold_;
    double slow_threshold_;
    double obstacle_zone_y_;
};

#endif // VELOCITY_CONTROLLER_HPP

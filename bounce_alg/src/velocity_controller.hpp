#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include <pcl/point_types.h>

class VelocityController
{
public:
    VelocityController(
        double stop_thresh, double slow_thresh, double obstacle_zone_y,
        double linear_vel, double angular_vel)
        : stop_threshold_(stop_thresh), slow_threshold_(slow_thresh), obstacle_zone_y_(obstacle_zone_y),
        linear_vel_param_(linear_vel), angular_vel_param_(angular_vel) {}

    // Function to calculate the velocity command
    geometry_msgs::msg::Twist calculateVelocity(double nearestPointDistance, const pcl::PointXYZ &nearestPoint)
    {

        geometry_msgs::msg::Twist cmd_vel;

        double linear_vel = 0.0;
        double angular_vel = 0.0;

        if (nearestPointDistance < 1.1) {}
        else if (nearestPointDistance < stop_threshold_ && nearestPoint.x > 0 && abs(nearestPoint.y) < obstacle_zone_y_)
        {
            // Stop and turn
            angular_vel = (nearestPoint.y > 0) ? -angular_vel_param_ : angular_vel_param_; // Turn right if obstacle on the left, and vice versa
        }
        else if (nearestPointDistance < slow_threshold_)
        {
            // Slow down and adjust direction
            linear_vel = linear_vel_param_ * (nearestPointDistance / slow_threshold_);
            angular_vel = -angular_vel_param_ * 0.5 * (nearestPoint.y / slow_threshold_); // Proportional steering to right if obstacle on the left, and vice versa
        }
        else
        {
            // No critical obstacle, proceed with default parameters
            linear_vel = linear_vel_param_;
        }

        // Set velocities in the Twist message
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        return cmd_vel;
    }

private:
    double stop_threshold_, slow_threshold_, obstacle_zone_y_;
    double linear_vel_param_, angular_vel_param_;
};

#endif // VELOCITY_CONTROLLER_HPP

# Obstacle Avoidance Using Bounce Algorithm
**Overview** This package implements a custom obstacle avoidance algorithm using point cloud data from a 3D LiDAR sensor. The robot "bounces" between obstacles using a reactive control method.

# How It Works

1. The robot is initially commanded to move forward by publishing to the cmd_vel topic.
2. The package subscribes to the 3D LiDAR output (real or simulated).
3. The algorithm processes the point cloud data:
4. Filters out points belonging to the ground plane.
5. Identifies the nearest obstacle.
6. If an object is detected within a specified range (0.5–1 meter), the robot stops, turns around, and resumes moving forward when no obstacles are detected in front.

# How to Run the Package

1. Build the package:

```colcon build```

2. Source the setup

```source install/setup.bash```

3. Launch the node:

```ros2 launch bounce_alg bounce_alg_launch.py```

# Parameters

You can modify the obstacle detection threshold (e.g., 0.5 meters) in the algorithm's configuration to adjust the robot's stopping distance.
Further Customization The algorithm can be expanded for more advanced obstacle avoidance behaviors, such as path planning or using a different LiDAR configuration. It currently uses a simple **"bounce"** mechanism for obstacle avoidance.

# Obstacle Avoidance Using Bounce Algorithm  

## Overview  
This package implements a custom obstacle avoidance algorithm using point cloud data from a 3D LiDAR sensor. The robot uses a reactive control method to navigate, dynamically "bouncing" between obstacles to ensure safe movement.  

## How It Works  
1. The robot is commanded to move forward by publishing velocity commands to the `cmd_vel` topic.  
2. The package subscribes to 3D LiDAR output (real or simulated).  
3. The algorithm processes the point cloud data by:  
   - Filtering out ground points based on elevation, tolerance, and a plane-fitting approach.  
   - Filtering points above a specified height to ensure the robot can pass under obstacles.  
   - Identifying the nearest obstacle and calculating its position.  
4. Based on obstacle proximity:  
   - If within the **stop threshold**, the robot halts and turns away from the obstacle.  
   - If within the **slow threshold**, the robot slows down and adjusts its direction.  
   - If no obstacles are detected, the robot moves at full speed.  

## How to Run the Package  

### Build the Package:  
```  
colcon build  
```  
Debuging in VS code
``` 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
``` 

### Source the Setup:  
```  
source install/setup.bash  
```  

### Launch the Node:  
```  
ros2 launch bounce_alg bounce_alg_launch.py  
```  

## Parameters  
The package parameters are configurable in the `config.yaml` file. Here are the key parameters and their ideal values:  

### Velocity  
- **linear_vel**: Maximum linear velocity [default: `0.2` m/s]  
- **angular_vel**: Maximum angular velocity [default: `0.4` rad/s]  

### Ground Filtration  
- **ground_elevation**: Elevation of the ground plane relative to the robot [default: `0.1` m]  
- **ground_tolerance**: Angle tolerance to identify ground points [default: `0.1` sin]  
- **ground_treshold**: Distance threshold for ground points during plane fitting [default: `0.07` m]  

### Obstacle Avoidance  
- **above_filtration**: Maximum height for filtering points above the robot [default: `0.2` m]  
- **slow_threshold**: Distance at which the robot slows down [default: `1.2` m]  
- **stop_threshold**: Distance at which the robot stops [default: `2.5` m]  
- **obstacle_zone_y**: Lateral distance used to bound obstacle zones [default: `2.0` m]  

### Transformation  
- **translation**: Transformation vector for aligning LiDAR data to the robot frame (used only with the real robot) [default: `[0.15, 0.0, -0.262]`]  

### Simulation Mode  
- **simulation**: Boolean flag to switch between simulation and real-world operation [`true` or `false`]  

## Further Customization  
This algorithm can be extended for more complex behaviors such as path planning or integration with different LiDAR configurations. While it currently uses a simple "bounce" mechanism for obstacle avoidance, it provides a foundation for implementing more advanced navigation techniques.  

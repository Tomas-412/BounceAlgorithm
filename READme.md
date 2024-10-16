# Custom package to process cloud point data

As the name says using this algorithm the robot bounces between obstacles. 

# Sequence

1. Start by publishing a cmd_vel to move the robot forward
2. The package subscribes to the LiDAR 3D output either from real or simulated sensor.
3. Then the data are processed by algorithm to find a ground floor and all the points which belongs there are filtered out.
4. From the rest of the data will be found a nearest object, if it's closer then given parameter (0.5 - 1 meter) then the robot stops, turns around and there is nothing in front it will continue from the step 1

# Tutorial

Build the package using colcon build

Change the source using command ```source install/setup.bash```

Launch the package with python launch file --> ```ros2 launch bounce_alg bounce_alg_node.cpp```

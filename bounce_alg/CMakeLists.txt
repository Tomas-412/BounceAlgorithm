cmake_minimum_required(VERSION 3.8)
project(bounce_alg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    ${pcl_conversions_INCLUDE_DIRS}
                   )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add the executable
add_executable(bounce_alg_node 
src/bounce_alg_node.cpp 
src/velocity_controller.hpp
src/point_cloud_processor.hpp)

# Link against the required libraries
target_link_libraries(bounce_alg_node 
                    ${PCL_LIBRARIES}
                    ${pcl_conversions_LIBRARIES}
                    )

ament_target_dependencies(bounce_alg_node
  rclcpp
  PCL
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs)

# Install target
install(TARGETS
  bounce_alg_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

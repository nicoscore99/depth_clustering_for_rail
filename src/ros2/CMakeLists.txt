add_executable(cpp_executable robosense_depth_clustering_node.cpp)
ament_target_dependencies(cpp_executable rclcpp sensor_msgs std_msgs nav_msgs message_filters tf2 Eigen3 visualization_msgs)

install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)
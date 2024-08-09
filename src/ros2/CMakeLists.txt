add_executable(cpp_executable robosense_depth_clustering_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)

install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)
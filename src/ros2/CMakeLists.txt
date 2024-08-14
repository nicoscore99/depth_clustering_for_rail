add_executable(robosense_depth_clustering_node robosense_depth_clustering_node.cpp)

# add_library(robosense_depth_clustering_node SHARED robosense_depth_clustering_node.cpp)

target_include_directories(robosense_depth_clustering_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(robosense_depth_clustering_node
                      cloud
                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
                      rotating_calipers
                      viewer
                      ${MY_QT_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${CMAKE_THREAD_LIBS_INIT})

ament_target_dependencies(robosense_depth_clustering_node rclcpp sensor_msgs std_msgs nav_msgs message_filters 
                          tf2 Eigen3 visualization_msgs depth_clustering_for_rail_interfaces foxglove_msgs)

install(TARGETS
  robosense_depth_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)
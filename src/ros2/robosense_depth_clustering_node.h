#ifndef DEPTH_CLUSTERING_NODE_H_
#define DEPTH_CLUSTERING_NODE_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ground_removal/depth_ground_remover.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "clusterers/image_based_clusterer.h"

namespace depth_clustering {

class DepthClusteringNode : public rclcpp::Node,
                            public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
public:
    // Constructor
    DepthClusteringNode(const Radians& theta_separation_thes, const Radians& ground_remove_angle, 
                        std::unique_ptr<ProjectionParams> proj_params_ptr, int min_cluster_size, 
                        int max_cluster_size, int smooth_window_size);

    // Callback for handling new clusters
    void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int) override;

private:
    // Converts ROS PointCloud2 messages to custom Cloud type
    Cloud::Ptr RosCloudToCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Callback for PointCloud2 subscription
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Member variables
    Radians _theta_separation_thes;
    std::int16_t _min_cluster_size;
    std::int16_t _max_cluster_size;
    std::int16_t _smooth_window_size;
    const Radians& _ground_remove_angle;
    std::unique_ptr<ProjectionParams> _proj_params_ptr;
    DepthGroundRemover ground_remover;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber;
    rclcpp::CallbackGroup::SharedPtr cloud_callback_group;
};

}  // namespace depth_clustering

#endif  // DEPTH_CLUSTERING_NODE_H_

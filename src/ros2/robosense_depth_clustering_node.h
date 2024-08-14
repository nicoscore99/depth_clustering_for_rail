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
#include "pcl/surface/convex_hull.h"
#include "rotating_calipers/RotatingCalipers.h"
// #include "depth_clustering_for_rail_interfaces/msg/Cluster.hpp"
// #include "depth_clustering_for_rail_interfaces/msg/ClusterArray.hpp"
#include <depth_clustering_for_rail_interfaces/msg/cluster_array.hpp>
#include <depth_clustering_for_rail_interfaces/msg/cluster.hpp>
#include "foxglove_msgs/msg/scene_entity.hpp"
#include "foxglove_msgs/msg/scene_entities.hpp"
#include "foxglove_msgs/msg/scene_entity_deletion.hpp"
#include "foxglove_msgs/msg/cube_primitive.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/color.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace depth_clustering {

using std::unordered_map;
using std::vector;

class DepthClusteringNode : public rclcpp::Node,
                            public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
public:
    // Constructor
    DepthClusteringNode(const Radians theta_separation_thes, const Radians ground_remove_angle, 
                        std::unique_ptr<ProjectionParams> proj_params_ptr, 
                        int min_cluster_size, int max_cluster_size, int smooth_window_size);

    // Callback for handling new clusters
    void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int) override;

    // Public member variable to allow external access to the clusterer
    ImageBasedClusterer<LinearImageLabeler<>> clusterer;

    // Member variables
    const Radians _theta_separation_thes;
    const Radians _ground_remove_angle;
    std::int32_t _min_cluster_size;
    std::int32_t _max_cluster_size;
    std::int32_t _smooth_window_size;
    std::unique_ptr<ProjectionParams> _proj_params_ptr;

private:
    // Converts ROS PointCloud2 messages to custom Cloud type
    Cloud::Ptr RosCloudToCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Converts the clusters to a ClusterArray message 
    depth_clustering_for_rail_interfaces::msg::ClusterArray generate_cluster_array_msg(const std::unordered_map<uint16_t, Cloud>& clouds,
                                                                                   const std::vector<std::vector<float>>& bboxes);

    // Converts the clusters to a SceneUpdate message
    foxglove_msgs::msg::SceneUpdate generate_scene_update_msg(const std::vector<std::vector<float>>& bboxes);

    // Callback for PointCloud2 subscription
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::vector<float> generate_bbox(const Cloud& _cloud);

    // foxglove_msgs::msg::SceneEntities prev_scene_entities = foxglove_msgs::msg::SceneEntities();
    DepthGroundRemover ground_remover;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber;
    rclcpp::Publisher<depth_clustering_for_rail_interfaces::msg::ClusterArray>::SharedPtr cluster_array_publisher;
    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr scene_update_publisher;
    rclcpp::CallbackGroup::SharedPtr cloud_callback_group;
};

}  // namespace depth_clustering

#endif  // DEPTH_CLUSTERING_NODE_H_

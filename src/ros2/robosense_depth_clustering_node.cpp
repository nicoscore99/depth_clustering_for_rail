#include "./robosense_depth_clustering_node.h"
#include "projections/projection_params.h"
#include "./utils.h"
#include "tclap/CmdLine.h"
#include <utils/cloud.h>

namespace depth_clustering {

DepthClusteringNode::DepthClusteringNode(const Radians& theta_separation_thes, const Radians& ground_remove_angle, 
                                         std::unique_ptr<ProjectionParams> proj_params_ptr, 
                                         int min_cluster_size, int max_cluster_size, int smooth_window_size)
    : Node("depth_clustering_node"),
      AbstractClient<std::unordered_map<uint16_t, Cloud>>(),
      _theta_separation_thes(theta_separation_thes),
      _min_cluster_size(min_cluster_size),
      _max_cluster_size(max_cluster_size),
      _smooth_window_size(smooth_window_size),
      _ground_remove_angle(ground_remove_angle),
      _proj_params_ptr(std::move(proj_params_ptr)),
      ground_remover(*_proj_params_ptr, _ground_remove_angle, _smooth_window_size),
      clusterer(_theta_separation_thes, min_cluster_size, max_cluster_size) {


    cloud_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group;

    cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 
        rclcpp::SensorDataQoS(),
        std::bind(&DepthClusteringNode::cloud_callback, this, std::placeholders::_1),
        cloud_options);

    clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
    ground_remover.AddClient(&clusterer);
    clusterer.AddClient(this);
}

void DepthClusteringNode::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int) {

    std::vector<MinAreaRect> bboxes;

    for (const auto& kv : clouds) {
        const auto& cluster = kv.second;
        bboxes.push_back(generate_bbox(cluster));
    }

    // Do something with the bounding boxes
    // ...
}

Cloud::Ptr DepthClusteringNode::RosCloudToCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    uint32_t x_offset = msg->fields[0].offset;
    uint32_t y_offset = msg->fields[1].offset;
    uint32_t z_offset = msg->fields[2].offset;

    Cloud cloud;
    for (uint32_t point_start_byte = 0, counter = 0;
         point_start_byte < msg->data.size();
         point_start_byte += msg->point_step, ++counter) {
        RichPoint point;
        point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
        point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
        point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
        cloud.push_back(point);
    }

    Cloud::Ptr cloud_ptr = boost::make_shared<Cloud>(cloud);
    return cloud_ptr;
}

void DepthClusteringNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Cloud::Ptr cloud_ptr = RosCloudToCloud(msg);
    cloud_ptr->InitProjection(*_proj_params_ptr);
    ground_remover.OnNewObjectReceived(*cloud_ptr, 0);
}

MinAreaRect DepthClusteringNode::generate_bbox(const Cloud& _cloud) {
    std::vector<int> hull_idcs = ConvexHullIndices(_cloud);

    Cloud hull_points;
    for (int idx : hull_idcs) {
        hull_points.push_back(_cloud[idx]);
    }

    // Only take the x, y axis of the points
    std::vector<Point> pts;
    for (const auto& point : hull_points.points()) {
        pts.push_back(Point{point.x(), point.y()});
    }

    MinAreaRect res = RotatingCalipers::minAreaRect(pts);

    return res;
}

}  // namespace depth_clustering

int main(int argc, char* argv[]) {
    TCLAP::CmdLine cmd(
        "Subscribe to /velodyne_points topic and save clusters to disc.", ' ',
        "1.0");
    TCLAP::ValueArg<int> theta_separation_thes_arg(
        "", "angle",
        "Threshold angle. Below this value, the objects are separated", false, 10,
        "int");
    TCLAP::ValueArg<int> min_cluster_size_arg(
        "", "min_cluster_size", "Minimum cluster size to save", false, 10, "int");
    TCLAP::ValueArg<int> max_cluster_size_arg(
        "", "max_cluster_size", "Maximum cluster size to save", false, 100000, "int");
    TCLAP::ValueArg<int> smooth_window_size_arg(
        "", "smooth_window_size", "Size of the window for smoothing", false, 5, "int");
    TCLAP::ValueArg<int> ground_remove_angle_arg(
        "", "ground_remove_angle", "Angle to remove ground", false, 7, "int");
         
    cmd.add(theta_separation_thes_arg);
    cmd.add(min_cluster_size_arg);
    cmd.add(max_cluster_size_arg);
    cmd.add(smooth_window_size_arg);
    cmd.add(ground_remove_angle_arg);

    cmd.parse(argc, argv);

    depth_clustering::Radians theta_separation_thes = depth_clustering::Radians::FromDegrees(theta_separation_thes_arg.getValue());
    depth_clustering::Radians ground_remove_angle = depth_clustering::Radians::FromDegrees(ground_remove_angle_arg.getValue());

    std::unique_ptr<depth_clustering::ProjectionParams> proj_params_ptr = depth_clustering::ProjectionParams::ROBOSENSE();
    
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::shared_ptr<depth_clustering::DepthClusteringNode>(new depth_clustering::DepthClusteringNode(
        theta_separation_thes, ground_remove_angle, std::move(proj_params_ptr),
        min_cluster_size_arg.getValue(), max_cluster_size_arg.getValue(), smooth_window_size_arg.getValue()
    ));
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}

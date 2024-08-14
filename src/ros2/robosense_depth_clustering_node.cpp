#include "./robosense_depth_clustering_node.h"
#include "projections/projection_params.h"
#include "./utils.h"
#include "tclap/CmdLine.h"
#include <utils/cloud.h>


namespace depth_clustering {

DepthClusteringNode::DepthClusteringNode(const Radians theta_separation_thes, const Radians ground_remove_angle, 
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
      ground_remover(*_proj_params_ptr, ground_remove_angle, _smooth_window_size),
      clusterer(theta_separation_thes, min_cluster_size, max_cluster_size) {

    // Debug prints using ROS 2 logging
    RCLCPP_INFO(this->get_logger(), "Constructor called");
    RCLCPP_INFO(this->get_logger(), "_theta_separation_thes: %f", _theta_separation_thes.val());
    RCLCPP_INFO(this->get_logger(), "_ground_remove_angle: %f", _ground_remove_angle.val());
    RCLCPP_INFO(this->get_logger(), "_min_cluster_size: %d", _min_cluster_size);
    RCLCPP_INFO(this->get_logger(), "_max_cluster_size: %d", _max_cluster_size);
    RCLCPP_INFO(this->get_logger(), "_smooth_window_size: %d", _smooth_window_size);
    RCLCPP_INFO(this->get_logger(), "_proj_params_ptr is %s", _proj_params_ptr ? "not null" : "null");

    if (!_proj_params_ptr) {
        RCLCPP_ERROR(this->get_logger(), "proj_params_ptr is null");
        throw std::runtime_error("proj_params_ptr is null");
    }

    // print _ground_remove_angle value and _theta_separation_thes value
    fprintf(stderr, "INFO: _ground_remove_angle: %f\n", _ground_remove_angle.val());
    fprintf(stderr, "INFO: _theta_separation_thes: %f\n", _theta_separation_thes.val());

    cloud_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group;

    cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 
        rclcpp::SensorDataQoS(),
        std::bind(&DepthClusteringNode::cloud_callback, this, std::placeholders::_1),
        cloud_options);

    cluster_array_publisher = this->create_publisher<depth_clustering_for_rail_interfaces::msg::ClusterArray>(
        "depth_clustering/clusters", 10);

    scene_update_publisher = this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
        "scene_update", 10);

    clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
    ground_remover.AddClient(&clusterer);
    clusterer.AddClient(this);
}

depth_clustering_for_rail_interfaces::msg::ClusterArray DepthClusteringNode::generate_cluster_array_msg(const std::unordered_map<uint16_t, Cloud>& clouds, 
                                                                                                    const std::vector<std::vector<float>>& bboxes) {

    // Assert that the number of clusters and bounding boxes are the same
    assert(clouds.size() == bboxes.size());

    depth_clustering_for_rail_interfaces::msg::ClusterArray cluster_array;

    // Header message
    cluster_array.header.stamp = this->now();
    cluster_array.header.frame_id = "rslidar";
    
    auto cloud_it = clouds.begin();
    for (size_t i = 0; i < bboxes.size(); ++i, ++cloud_it) {

        const auto& kv = *cloud_it;

        // Cluster message
        depth_clustering_for_rail_interfaces::msg::Cluster cluster_msg;

        // Clusterr
        const auto& cluster = kv.second;
        
        // PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;

        // Fill the PointCloud2 message
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "rslidar";
        cloud_msg.height = 1;
        cloud_msg.width = cluster.size();
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;
        cloud_msg.point_step = 16;
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;
        cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

        for (int i = 0; i < cluster.size(); ++i) {
            const auto& point = cluster[i];
            float x = point.x();
            float y = point.y();
            float z = point.z();
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + cloud_msg.fields[0].offset], &x, sizeof(float));
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + cloud_msg.fields[1].offset], &y, sizeof(float));
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + cloud_msg.fields[2].offset], &z, sizeof(float));
        }

        // Cluster message
        cluster_msg.pointcloud = cloud_msg;

        // Bounding box message
        cluster_msg.bbox = bboxes[i];
        cluster_array.clusters.push_back(cluster_msg);

    }
    return cluster_array;
}

foxglove_msgs::msg::SceneUpdate DepthClusteringNode::generate_scene_update_msg(const std::vector<std::vector<float>>& bboxes) {

    foxglove_msgs::msg::SceneUpdate scene_update;

    for (size_t i = 0; i < bboxes.size(); ++i) {
        // SceneEntity message
        foxglove_msgs::msg::SceneEntity scene_entity;

        scene_entity.timestamp = this->now();
        scene_entity.id = i;
        scene_entity.frame_id = "rslidar";
        builtin_interfaces::msg::Duration scene_entity_lifetime;
        scene_entity_lifetime.sec = 1;
        scene_entity.lifetime = scene_entity_lifetime;
        scene_entity.frame_locked = true;

        // Bounding box
        const auto& bbox = bboxes[i];

        // CubePrimitive message
        foxglove_msgs::msg::CubePrimitive cube_primitive;

        geometry_msgs::msg::Pose pose;
        geometry_msgs::msg::Point position;

        position.x = bbox[0];
        position.y = bbox[1];
        position.z = bbox[2];

        pose.position = position;

        geometry_msgs::msg::Quaternion orientation;

        tf2::Quaternion q;
        q.setRPY(0, 0, bbox[6]);
        orientation.x = q.x();
        orientation.y = q.y();
        orientation.z = q.z();
        orientation.w = q.w();

        pose.orientation = orientation;

        cube_primitive.pose = pose;

        geometry_msgs::msg::Vector3 cube_size;

        cube_size.x = bbox[3];
        cube_size.y = bbox[4];
        cube_size.z = bbox[5];

        cube_primitive.size = cube_size;

        // Color message
        foxglove_msgs::msg::Color color;
        color.r = 255;
        color.g = 255;
        color.b = 255;
        color.a = 0.3;

        cube_primitive.color = color;

        // Add the CubePrimitive to the SceneEntity
        scene_entity.cubes.push_back(cube_primitive);

        // Add the SceneEntity to the SceneUpdate
        scene_update.entities.push_back(scene_entity);

        // prev_scene_entities = scene_update.entities;
    }

    // SceneEntityDeletion message
    foxglove_msgs::msg::SceneEntityDeletion scene_entity_deletion;
    scene_entity_deletion.type = 1;

    scene_update.deletions.push_back(scene_entity_deletion);

    return scene_update;
}

void DepthClusteringNode::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int) {

    std::vector<std::vector<float>> bboxes;

    for (const auto& kv : clouds) {
        const auto& cluster = kv.second;
        bboxes.push_back(generate_bbox(cluster));
    }
    depth_clustering_for_rail_interfaces::msg::ClusterArray cluster_array = generate_cluster_array_msg(clouds, bboxes);
    cluster_array_publisher->publish(cluster_array);

    foxglove_msgs::msg::SceneUpdate scene_update = generate_scene_update_msg(bboxes);
    scene_update_publisher->publish(scene_update);
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

std::vector<float> DepthClusteringNode::generate_bbox(const Cloud& _cloud) {
    std::vector<int> hull_idcs = ConvexHullIndices(_cloud);

    Cloud hull_points;
    for (int idx : hull_idcs) {
        hull_points.push_back(_cloud[idx]);
    }

    // Only take the x, y axis of the points
    std::vector<rotating_calipers::Point> pts;
    for (const auto& point : hull_points.points()) {
        pts.push_back(rotating_calipers::Point{point.x(), point.y()});
    }

    rotating_calipers::MinAreaRect res = rotating_calipers::RotatingCalipers::minAreaRect(pts);

    // Bounding box is of structure x, y, z, w, l, h, yaw

    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    for (const auto& point : _cloud.points()) {
        z_min = std::min(z_min, point.z());
        z_max = std::max(z_max, point.z());
    }

    // Add the z values to the bounding box
    float x_center = res.center.x;
    float y_center = res.center.y;
    float z_center = (z_min + z_max) / 2.0f;
    float width = res.width;
    float length = res.height;
    float height = z_max - z_min;
    float yaw = res.angle_height;

    std::vector<float> bbox = {x_center, y_center, z_center, width, length, height, yaw};

    // Print the bounding box
    fprintf(stderr, "INFO: Bounding box: x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, yaw: %f\n", x_center, y_center, z_center, width, length, height, yaw);

    return bbox;
}

}  // namespace depth_clustering

int main(int argc, char* argv[]) {
    TCLAP::CmdLine cmd(
        "Subscribe to /velodyne_points topic and save clusters to disc.", ' ',
        "1.0");
    TCLAP::ValueArg<float> theta_separation_thes_arg(
        "", "angle",
        "Threshold angle. Below this value, the objects are separated", false, 10,
        "float");
    TCLAP::ValueArg<float> ground_remove_angle_arg(
        "", "ground_remove_angle", "Angle to remove ground", false, 7, "float");
    TCLAP::ValueArg<int> min_cluster_size_arg(
        "", "min_cluster_size", "Minimum cluster size to save", false, 15, "int");
    TCLAP::ValueArg<int> max_cluster_size_arg(
        "", "max_cluster_size", "Maximum cluster size to save", false, 100000, "int");
    TCLAP::ValueArg<int> smooth_window_size_arg(
        "", "smooth_window_size", "Size of the window for smoothing", false, 5, "int");
         
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

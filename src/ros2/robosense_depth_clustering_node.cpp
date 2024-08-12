#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/projection_params.h"
#include "projections/spherical_projection.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"
#include "tclap/CmdLine.h"


#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace depth_clustering {

using std::array;
using std::string;
using std::to_string;
using std::vector;

using std::lock_guard;
using std::map;
using std::mutex;
using std::string;
using std::thread;
using std::unordered_map;
using std::vector;

template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

class DepthClusteringNode : public rclcpp::Node,
                            public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
  
  public:

    Radians _theta_separation_thes;
    std::int16_t _min_cluster_size;
    std::int16_t _max_cluster_size;
    std::int16_t _smooth_window_size;
    std::int16_t _ground_remove_angle;
    std::unique_ptr<ProjectionParams> _proj_params_ptr;

    DepthGroundRemover ground_remover;
        
    DepthClusteringNode(const Radians& theta_separation_thes, const Radians& ground_remove_angle, std::unique_ptr<ProjectionParams> proj_params_ptr,
                        const int min_cluster_size, const int max_cluster_size, int smooth_window_size)
        : Node("depth_clustering_node"),
          AbstractClient<std::unordered_map<uint16_t, Cloud>>(),
          _theta_separation_thes(theta_separation_thes),
          _min_cluster_size(min_cluster_size),
          _max_cluster_size(max_cluster_size),
          _smooth_window_size(smooth_window_size),
          _ground_remove_angle(ground_remove_angle.val()),
          _proj_params_ptr(std::move(proj_params_ptr)),
          ground_remover(*proj_params_ptr, ground_remove_angle, smooth_window_size)  // Initialized here
    {

      cloud_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions cloud_options;
      cloud_options.callback_group = cloud_callback_group;
		
      cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 
        rclcpp::SensorDataQoS(),
        std::bind(&DepthClusteringNode::cloud_callback, this, std::placeholders::_1),
        cloud_options);

		  ImageBasedClusterer<LinearImageLabeler<>> clusterer(_theta_separation_thes, _ground_remove_angle, _smooth_window_size);

      ground_remover.AddClient(&clusterer);
      clusterer.AddClient(this);
    }

    void OnNewObjectReceived(const unordered_map<uint16_t, Cloud>& clouds, const int) {
      fprintf(stderr, "INFO: received %lu clusters\n", clouds.size());
    }

	private:

    // void generate_cubes(const std::unordered_map<uint16_t, Cloud>& clusters) {

    //   // 

    //   for (const auto& cluster : clusters) {
    //     const auto& cluster = kv.second;
    //     Eigen::Vector3f center = Eigen::Vector3f::Zero();
    //     Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    //     Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
    //                               std::numeric_limits<float>::lowest(),
    //                               std::numeric_limits<float>::lowest());
    //     Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
    //                               std::numeric_limits<float>::max(),
    //                               std::numeric_limits<float>::max());
    //     for (const auto& point : cluster.points()) {
    //       center = center + point.AsEigenVector();
    //       min_point << std::min(min_point.x(), point.x()),
    //           std::min(min_point.y(), point.y()),
    //           std::min(min_point.z(), point.z());
    //       max_point << std::max(max_point.x(), point.x()),
    //           std::max(max_point.y(), point.y()),
    //           std::max(max_point.z(), point.z());
    //     }
    //     center /= cluster.size();
    //     if (min_point.x() < max_point.x()) {
    //       extent = max_point - min_point;
    //     }

    //     // Create an array of with content x, y, z, l, w, h, yaw

    //     std::array 

    //   }
    // }

		Cloud::Ptr RosCloudToCloud(
				const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			uint32_t x_offset = msg->fields[0].offset;
			uint32_t y_offset = msg->fields[1].offset;
			uint32_t z_offset = msg->fields[2].offset;
			uint32_t ring_offset = msg->fields[4].offset;

			Cloud cloud;
			for (uint32_t point_start_byte = 0, counter = 0;
					point_start_byte < msg->data.size();
					point_start_byte += msg->point_step, ++counter) {
				RichPoint point;
				point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
				point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
				point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
				point.ring() = BytesTo<uint16_t>(msg->data, point_start_byte + ring_offset);
				// point.z *= -1;  // hack
				cloud.push_back(point);
			}

			Cloud::Ptr cloud_ptr = boost::make_shared<Cloud>(cloud);
      return cloud_ptr;
		}

	  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	    Cloud::Ptr cloud_ptr = RosCloudToCloud(msg);
			cloud_ptr -> InitProjection(*_proj_params_ptr);
			this->ground_remover.OnNewObjectReceived(*cloud_ptr, 0);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber;
    rclcpp::CallbackGroup::SharedPtr cloud_callback_group;

};


int main(int argc, char* argv[]) {
  TCLAP::CmdLine cmd(
      "Subscribe to /velodyne_points topic and save clusters to disc.", ' ',
      "1.0");
  TCLAP::ValueArg<int> theta_separation_thes_arg(
      "", "angle",
      "Threshold angle. Below this value, the objects are separated", false, 10,
      "int");
  TCLAP::ValueArg<int> num_beams_arg(
      "", "num_beams", "Num of vertical beams in laser. If number 64 is chosen, "
                      "HDL-64 projection params are used. Else Robosense per default",
      true, 0, "int");
  TCLAP::ValueArg<int> min_cluster_size_arg(
      "", "min_cluster_size", "Minimum cluster size to save", false, 20, "int");
  TCLAP::ValueArg<int> max_cluster_size_arg(
        "", "max_cluster_size", "Maximum cluster size to save", false, 100000, "int");
  TCLAP::ValueArg<int> smooth_window_size_arg(
      "", "smooth_window_size", "Size of the window for smoothing", false, 9, "int");
  TCLAP::ValueArg<int> ground_remove_angle_arg(
        "", "ground_remove_angle", "Angle to remove ground", false, 7, "int");
         
  cmd.add(theta_separation_thes_arg);
  cmd.add(num_beams_arg);
  cmd.add(min_cluster_size_arg);
  cmd.add(max_cluster_size_arg);
  cmd.add(smooth_window_size_arg);
  cmd.add(ground_remove_angle_arg);

  cmd.parse(argc, argv);

  Radians theta_separation_thes = Radians::FromDegrees(theta_separation_thes_arg.getValue());
  Radians ground_remove_angle = Radians::FromDegrees(ground_remove_angle_arg.getValue());

  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;
  switch (num_beams_arg.getValue()) {
    case 64:
      proj_params_ptr = ProjectionParams::HDL_64();
      break;
    default:
      proj_params_ptr = ProjectionParams::ROBOSENSE();
      break;
  }
  if (!proj_params_ptr) {
    fprintf(stderr,
            "ERROR: wrong number of beams: %d. Should be in {64, 0}\n",
            num_beams_arg.getValue());
    exit(1);
  }

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::shared_ptr<DepthClusteringNode>(new DepthClusteringNode(
    theta_separation_thes, ground_remove_angle, std::move(proj_params_ptr),
    min_cluster_size_arg.getValue(), max_cluster_size_arg.getValue(), smooth_window_size_arg.getValue()
  ));
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}

}

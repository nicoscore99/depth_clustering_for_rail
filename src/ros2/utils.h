// utils.h
#ifndef UTILS_H_
#define UTILS_H_

#include <vector>
#include <cstdint>
#include <algorithm>

#include <utils/cloud.h>

template <class T>
T BytesTo(const std::vector<uint8_t>& data, uint32_t start_idx) {
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

namespace depth_clustering {

std::vector<int> ConvexHullIndices(const depth_clustering::Cloud& cloud) {

  float min_z{std::numeric_limits<float>::max()};
  float max_z{std::numeric_limits<float>::lowest()};
  std::vector<cv::Point2f> cv_points;
  cv_points.reserve(cloud.size());
  std::vector<int> hull_indices;
  for (const auto& point : cloud.points()) {
    cv_points.emplace_back(cv::Point2f{point.x(), point.y()});
  }
  cv::convexHull(cv_points, hull_indices);  
  // return the indices of the convex hull points
  return hull_indices;
}
  
}  // namespace depth_clustering

#endif  // UTILS_H_
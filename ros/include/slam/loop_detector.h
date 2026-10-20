#pragma once

#ifndef KISS_MATCHER_LOOP_DETECTOR_H
#define KISS_MATCHER_LOOP_DETECTOR_H

///// C++ common headers
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <solid/database.hpp>
#include <solid/extractor.hpp>

#include "rclcpp/rclcpp.hpp"
#include "slam/loop_types.hpp"
#include "slam/pose_graph_node.hpp"
#include "slam/utils.hpp"

#define LOOP_CANDIDATE_NOT_FOUND -1

namespace kiss_matcher {

struct LoopDetectorConfig {
  bool verbose_ = false;
  bool enable_  = false;
  // Keyframes acquired within this time window from the query are excluded from the database
  double timediff_threshold_ = 90.0;
  // Minimum cosine similarity between R-SOLiDs to be considered as a loop candidate
  double similarity_threshold_ = 0.95;
  // Maximum number of loop candidates to be verified by registration per query
  int num_candidates_ = 1;

  solid::Config solid_config_;
};

class LoopDetector {
 private:
  LoopDetectorConfig config_;
  rclcpp::Logger logger_;

  solid::Extractor extractor_;
  solid::Database database_;

  // Descriptors are cached per keyframe index and inserted into the database lazily,
  // i.e., only once they are older than `timediff_threshold_` w.r.t. the query.
  std::vector<solid::Descriptor> descriptors_;
  std::vector<double> timestamps_;
  size_t num_inserted_ = 0;

  solid::Descriptor extractDescriptor(const pcl::PointCloud<PointType> &scan) const;

 public:
  explicit LoopDetector(const LoopDetectorConfig &config, const rclcpp::Logger &logger);
  ~LoopDetector();
  LoopIdxPairs fetchLoopCandidates(const PoseGraphNode &query_frame,
                                   const std::vector<PoseGraphNode> &keyframes);
};
}  // namespace kiss_matcher
#endif  // KISS_MATCHER_LOOP_DETECTOR_H

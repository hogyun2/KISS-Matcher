#include "slam/loop_detector.h"

using namespace kiss_matcher;

LoopDetector::LoopDetector(const LoopDetectorConfig &config, const rclcpp::Logger &logger)
    : config_(config),
      logger_(logger),
      extractor_(config.solid_config_),
      database_(config.solid_config_) {}

LoopDetector::~LoopDetector() {}

solid::Descriptor LoopDetector::extractDescriptor(const pcl::PointCloud<PointType> &scan) const {
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(scan.size());
  for (const auto &pt : scan.points) {
    pts.emplace_back(pt.x, pt.y, pt.z);
  }
  return extractor_.extract(pts);
}

LoopIdxPairs LoopDetector::fetchLoopCandidates(const PoseGraphNode &query_frame,
                                               const std::vector<PoseGraphNode> &keyframes) {
  LoopIdxPairs loop_idx_pairs;
  if (!config_.enable_) {
    return loop_idx_pairs;
  }

  // The detector is triggered by a timer, so some keyframes may be skipped as queries.
  // Still, their descriptors are needed so that they can be retrieved later.
  const size_t query_idx = query_frame.idx_;
  while (descriptors_.size() <= query_idx && descriptors_.size() < keyframes.size()) {
    const auto &keyframe = keyframes[descriptors_.size()];
    descriptors_.emplace_back(extractDescriptor(keyframe.scan_));
    timestamps_.emplace_back(keyframe.timestamp_);
  }
  if (descriptors_.size() <= query_idx) {
    return loop_idx_pairs;
  }

  while (num_inserted_ < query_idx &&
         query_frame.timestamp_ - timestamps_[num_inserted_] > config_.timediff_threshold_) {
    database_.add(num_inserted_, descriptors_[num_inserted_]);
    ++num_inserted_;
  }
  if (database_.empty()) {
    return loop_idx_pairs;
  }

  const auto &candidates = database_.query(descriptors_[query_idx]);
  for (const auto &candidate : candidates) {
    if (static_cast<int>(loop_idx_pairs.size()) >= config_.num_candidates_ ||
        candidate.score < config_.similarity_threshold_) {
      break;
    }
    loop_idx_pairs.emplace_back(query_idx, candidate.id);
    if (config_.verbose_) {
      RCLCPP_INFO(logger_,
                  "SOLiD candidate: %zu <-> %zu (similarity: %.4f)",
                  query_idx,
                  candidate.id,
                  candidate.score);
    }
  }
  return loop_idx_pairs;
}

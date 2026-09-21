/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/pose_graph_trimmer.h"

#include <algorithm>
#include <limits>

#include "cartographer/mapping/pose_graph_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PureLocalizationTrimmer::PureLocalizationTrimmer(const int trajectory_id,
                                                 const int num_submaps_to_keep)
    : trajectory_id_(trajectory_id), num_submaps_to_keep_(num_submaps_to_keep) {
  CHECK_GE(num_submaps_to_keep, 2) << "Cannot trim with less than 2 submaps";
}

PureLocalizationTrimmer::PureLocalizationTrimmer(
    const int trajectory_id, const int num_submaps_to_keep,
    const bool keep_uncovered, const double coverage_resolution,
    const double coverage_radius)
    : trajectory_id_(trajectory_id),
      num_submaps_to_keep_(num_submaps_to_keep),
      keep_uncovered_(keep_uncovered),
      coverage_resolution_(coverage_resolution > 0. ? coverage_resolution : 1.),
      coverage_radius_(coverage_radius > 0. ? coverage_radius : 12.) {
  CHECK_GE(num_submaps_to_keep, 2) << "Cannot trim with less than 2 submaps";
}

void PureLocalizationTrimmer::MarkCovered(const double x, const double y) {
  const int r = static_cast<int>(coverage_radius_ / coverage_resolution_);
  const int cx = static_cast<int>((x - origin_x_) / coverage_resolution_);
  const int cy = static_cast<int>((y - origin_y_) / coverage_resolution_);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r * r) continue;
      const int ix = cx + dx;
      const int iy = cy + dy;
      if (ix < 0 || iy < 0 || ix >= coverage_width_ || iy >= coverage_height_) {
        continue;
      }
      covered_[static_cast<size_t>(iy) * coverage_width_ + ix] = true;
    }
  }
}

bool PureLocalizationTrimmer::IsCovered(const double x, const double y) const {
  const int ix = static_cast<int>((x - origin_x_) / coverage_resolution_);
  const int iy = static_cast<int>((y - origin_y_) / coverage_resolution_);
  if (ix < 0 || iy < 0 || ix >= coverage_width_ || iy >= coverage_height_) {
    return false;
  }
  return covered_[static_cast<size_t>(iy) * coverage_width_ + ix];
}

// Fork (2026-09-21). Off-map retention for pure localization.
//
// OFFLINE VERIFIED, ONLINE UNVALIDATED: exercised in offline map builds and
// replay only, never on the robot. It is also off by default in the lua
// (keep_uncovered = false), so nothing runs this path unless asked.
//
// Coverage is "ground the robot has driven before": the node positions of the
// FROZEN trajectories, dilated by coverage_radius. That is the thing
// localization actually depends on, it needs no submap grids, and it costs one
// pass over the loaded nodes at startup instead of per-pass work.
//
// A live submap whose origin falls inside that region is redundant and trimmed
// as usual, so on stored ground the live trajectory stays capped at
// max_submaps_to_keep and the CPU profile is unchanged. A live submap outside
// it is spared and folded into the coverage, so the run keeps what it maps off
// the stored map, and a second pass over that same new ground is trimmed rather
// than accumulating another copy.
//
// Deliberately NOT OverlappingSubmapsTrimmer2D, which answers a similar
// question by rebuilding a std::map keyed on every cell of every submap on
// EVERY pass while holding the pose graph mutex. Measured on the 382-submap
// campus map that is ~900M iterations and a tree on the order of 10 GB per
// pass; it wedges the node rather than merely costing time (work queue to 39k
// items, no constraints, map->odom stuck at its seed).
bool PureLocalizationTrimmer::IsRedundant(const SubmapId& submap_id,
                                          Trimmable* const pose_graph) {
  const auto& nodes = pose_graph->GetTrajectoryNodes();
  if (!coverage_built_) {
    coverage_built_ = true;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = min_x;
    double max_x = -min_x;
    double max_y = -min_x;
    int n_frozen = 0;
    for (const auto& node : nodes) {
      if (!pose_graph->IsFrozen(node.id.trajectory_id)) continue;
      const Eigen::Vector3d t = node.data.global_pose.translation();
      min_x = std::min(min_x, t.x());
      max_x = std::max(max_x, t.x());
      min_y = std::min(min_y, t.y());
      max_y = std::max(max_y, t.y());
      ++n_frozen;
    }
    if (n_frozen == 0) {
      // Nothing loaded: every live submap is "uncovered", which would keep the
      // whole trajectory forever. Fall back to the stock behaviour instead.
      LOG(WARNING) << "PureLocalizationTrimmer: keep_uncovered is set but no "
                      "frozen trajectory is loaded; trimming as usual.";
      coverage_width_ = coverage_height_ = 0;
      return true;
    }
    origin_x_ = min_x - coverage_radius_;
    origin_y_ = min_y - coverage_radius_;
    coverage_width_ = static_cast<int>((max_x - min_x + 2 * coverage_radius_) /
                                       coverage_resolution_) + 1;
    coverage_height_ = static_cast<int>((max_y - min_y + 2 * coverage_radius_) /
                                        coverage_resolution_) + 1;
    covered_.assign(static_cast<size_t>(coverage_width_) * coverage_height_,
                    false);
    for (const auto& node : nodes) {
      if (!pose_graph->IsFrozen(node.id.trajectory_id)) continue;
      const Eigen::Vector3d t = node.data.global_pose.translation();
      MarkCovered(t.x(), t.y());
    }
    LOG(INFO) << "PureLocalizationTrimmer: coverage from " << n_frozen
              << " frozen nodes, " << coverage_width_ << "x" << coverage_height_
              << " cells at " << coverage_resolution_ << " m, radius "
              << coverage_radius_ << " m";
  }
  if (coverage_width_ == 0) return true;

  // Judge the submap by the nodes inserted into it, not by its origin: a submap
  // straddling the edge of the stored map is kept, which is the conservative
  // direction.
  bool any_uncovered = false;
  for (const auto& constraint : pose_graph->GetConstraints()) {
    if (constraint.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) continue;
    if (!(constraint.submap_id == submap_id)) continue;
    const auto it = nodes.find(constraint.node_id);
    if (it == nodes.end()) continue;
    const Eigen::Vector3d t = it->data.global_pose.translation();
    if (!IsCovered(t.x(), t.y())) {
      any_uncovered = true;
      break;
    }
  }
  if (!any_uncovered) return true;

  for (const auto& constraint : pose_graph->GetConstraints()) {
    if (constraint.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) continue;
    if (!(constraint.submap_id == submap_id)) continue;
    const auto it = nodes.find(constraint.node_id);
    if (it == nodes.end()) continue;
    const Eigen::Vector3d t = it->data.global_pose.translation();
    MarkCovered(t.x(), t.y());
  }
  LOG(INFO) << "PureLocalizationTrimmer: keeping live submap " << submap_id
            << ", it reaches ground the stored map does not cover";
  return false;
}

void PureLocalizationTrimmer::Trim(Trimmable* const pose_graph) {
  if (pose_graph->IsFinished(trajectory_id_)) {
    num_submaps_to_keep_ = 0;
  }

  auto submap_ids = pose_graph->GetSubmapIds(trajectory_id_);
  for (std::size_t i = 0; i + num_submaps_to_keep_ < submap_ids.size(); ++i) {
    // Fork (2026-09-21): spare a submap that reaches ground the stored map does
    // not describe, so driving off the map keeps what it builds. On stored
    // ground this is always false and the stock behaviour is unchanged.
    // Skipped entirely when the trajectory is finished, where num_submaps_to_
    // keep_ is 0 and everything must go.
    if (keep_uncovered_ && num_submaps_to_keep_ > 0 &&
        !IsRedundant(submap_ids.at(i), pose_graph)) {
      continue;
    }
    pose_graph->TrimSubmap(submap_ids.at(i));
  }

  if (num_submaps_to_keep_ == 0) {
    finished_ = true;
    pose_graph->SetTrajectoryState(
        trajectory_id_, PoseGraphInterface::TrajectoryState::DELETED);
  }
}

bool PureLocalizationTrimmer::IsFinished() { return finished_; }

}  // namespace mapping
}  // namespace cartographer

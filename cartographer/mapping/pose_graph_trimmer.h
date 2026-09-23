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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_

#include <set>
#include <vector>

#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"

namespace cartographer {
namespace mapping {

// Implemented by the pose graph to provide thread-safe access to functions for
// trimming the graph.
class Trimmable {
 public:
  virtual ~Trimmable() {}

  virtual int num_submaps(int trajectory_id) const = 0;

  virtual std::vector<SubmapId> GetSubmapIds(int trajectory_id) const = 0;
  // Returns finished submaps with optimized poses only.
  virtual MapById<SubmapId, PoseGraphInterface::SubmapData>
  GetOptimizedSubmapData() const = 0;
  virtual const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const = 0;
  virtual const std::vector<PoseGraphInterface::Constraint>& GetConstraints()
      const = 0;

  // Trim 'submap_id' and corresponding intra-submap nodes. They
  // will no longer take part in scan matching, loop closure, visualization.
  // The numbering remains unchanged.
  virtual void TrimSubmap(const SubmapId& submap_id) = 0;

  // Checks if the given trajectory is finished or not.
  virtual bool IsFinished(int trajectory_id) const = 0;

  // Fork (2026-09-21): checks if the given trajectory is FROZEN, i.e. was
  // loaded from a stored state rather than built live. FINISHED is a
  // different thing and does not imply it, so IsFinished cannot answer this.
  // Needed by OverlappingSubmapsTrimmer2D's localization mode, which must
  // rank stored submaps above live ones and must never trim a stored one.
  virtual bool IsFrozen(int trajectory_id) const = 0;

  // Sets the state for a specific trajectory.
  virtual void SetTrajectoryState(
      int trajectory_id, PoseGraphInterface::TrajectoryState state) = 0;
};

// An interface to implement algorithms that choose how to trim the pose graph.
class PoseGraphTrimmer {
 public:
  virtual ~PoseGraphTrimmer() {}

  // Called once after each pose graph optimization.
  virtual void Trim(Trimmable* pose_graph) = 0;

  // Checks if this trimmer is in a terminatable state.
  virtual bool IsFinished() = 0;
};

// Keeps the last 'num_submaps_to_keep' of the trajectory with 'trajectory_id'
// to implement localization without mapping.
class PureLocalizationTrimmer : public PoseGraphTrimmer {
 public:
  PureLocalizationTrimmer(int trajectory_id, int num_submaps_to_keep);
  // Fork (2026-09-21): off-map retention. See the proto for what these mean.
  PureLocalizationTrimmer(int trajectory_id, int num_submaps_to_keep,
                          bool keep_uncovered, double coverage_resolution,
                          double coverage_radius, double keep_radius);
  ~PureLocalizationTrimmer() override {}

  void Trim(Trimmable* pose_graph) override;
  bool IsFinished() override;

 private:
  // True when the stored map already describes this submap's ground, so
  // dropping it loses nothing. Builds the coverage bitmap on first use and
  // folds spared live submaps into it afterwards.
  bool IsRedundant(const SubmapId& submap_id, Trimmable* pose_graph);
  void MarkCovered(double x, double y, double radius);
  bool IsCovered(double x, double y) const;

  const int trajectory_id_;
  int num_submaps_to_keep_;
  bool finished_ = false;

  const bool keep_uncovered_ = false;
  const double coverage_resolution_ = 1.0;
  const double coverage_radius_ = 12.0;
  // Dilation used when a KEPT live submap is folded into the coverage. Small
  // on purpose: it decides whether a later submap re-drives the same new
  // ground, not whether it is near the old map. Using coverage_radius here
  // made the next consecutive submap (6 m on) redundant every time.
  const double keep_radius_ = 3.0;
  // Submaps spared once are spared for the run. Re-testing them found their
  // own folded-in nodes and trimmed them on the next pass (bug, 2026-09-22).
  std::set<SubmapId> spared_;
  bool coverage_built_ = false;
  double origin_x_ = 0., origin_y_ = 0.;
  int coverage_width_ = 0, coverage_height_ = 0;
  std::vector<bool> covered_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_

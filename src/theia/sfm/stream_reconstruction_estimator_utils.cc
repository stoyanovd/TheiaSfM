// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/sfm/stream_reconstruction_estimator_utils.h"

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.h"
#include "theia/sfm/camera/reprojection_error.h"
#include "theia/sfm/global_pose_estimation/nonlinear_position_estimator.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/map_util.h"
#include "theia/util/threadpool.h"

namespace theia {

std::tuple<int, int, int> NumTracksLargeDiff(
    const Reconstruction& reconstruction, ViewId start_view_id, int diff_ids) {
  int num_total = 0;
  int num_large_diff = 0;
  int num_large_diff_est = 0;
  for (const theia::TrackId track_id : reconstruction.TrackIds()) {
    const auto* track = reconstruction.Track(track_id);
    if (track == nullptr) {
      continue;
    }
    auto leader_view_ids = track->ViewIds();
    theia::ViewId min_view_id =
        *std::min_element(leader_view_ids.begin(), leader_view_ids.end());
    theia::ViewId max_view_id =
        *std::max_element(leader_view_ids.begin(), leader_view_ids.end());

    if (max_view_id > start_view_id) {
      num_total++;
      num_large_diff += (max_view_id - min_view_id > diff_ids);
      num_large_diff_est +=
          (max_view_id - min_view_id > diff_ids) && track->IsEstimated();
    }
  }

  return std::make_tuple(num_total, num_large_diff, num_large_diff_est);
}

}  // namespace theia

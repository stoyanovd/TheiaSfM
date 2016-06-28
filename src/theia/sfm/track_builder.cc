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

#include "theia/sfm/track_builder.h"

#include <stdint.h>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "theia/math/graph/connected_components.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"
#include "theia/util/map_util.h"

namespace theia {

TrackBuilder::TrackBuilder(const int min_track_length,
                           const int max_track_length)
    : num_features_(0), min_track_length_(min_track_length) {
  connected_components_.reset(
      new ConnectedComponents<uint64_t>(max_track_length));
}

TrackBuilder::~TrackBuilder() {}

void TrackBuilder::AddFeatureCorrespondence(const ViewId view_id1,
                                            const Feature& feature1,
                                            const ViewId view_id2,
                                            const Feature& feature2) {
  CHECK_NE(view_id1, view_id2)
      << "Cannot add 2 features from the same image as a correspondence for "
         "track generation.";

  const auto image_feature1 = std::make_pair(view_id1, feature1);
  const auto image_feature2 = std::make_pair(view_id2, feature2);

  const int feature1_id = FindOrInsert(image_feature1);
  const int feature2_id = FindOrInsert(image_feature2);

  connected_components_->AddEdge(feature1_id, feature2_id);
}

void TrackBuilder::BuildTracks(Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);
  CHECK_EQ(reconstruction->NumTracks(), 0);

  // Build a reverse map mapping feature ids to ImageNameFeaturePairs.
  std::unordered_map<uint64_t, const std::pair<ViewId, Feature>*> id_to_feature;
  id_to_feature.reserve(features_.size());
  for (const auto& feature : features_) {
    InsertOrDie(&id_to_feature, feature.second, &feature.first);
  }

  // Extract all connected components.
  std::unordered_map<uint64_t, std::unordered_set<uint64_t> > components;
  connected_components_->Extract(&components);

  // Each connected component is a track. Add all tracks to the reconstruction.
  int num_small_tracks = 0;
  int num_inconsistent_features = 0;
  for (const auto& component : components) {
    // Skip singleton tracks.
    if (component.second.size() < min_track_length_) {
      ++num_small_tracks;
      continue;
    }

    std::vector<std::pair<ViewId, Feature> > track;
    track.reserve(component.second.size());

    // Add all features in the connected component to the track.
    std::unordered_set<ViewId> view_ids;
    for (const auto& feature_id : component.second) {
      const auto& feature_to_add = *FindOrDie(id_to_feature, feature_id);

      // Do not add the feature if the track already contains a feature from the
      // same image.
      if (!InsertIfNotPresent(&view_ids, feature_to_add.first)) {
        ++num_inconsistent_features;
        continue;
      }

      track.emplace_back(feature_to_add);
    }

    TrackId added_track_id = reconstruction->AddTrack(track);
    CHECK_NE(added_track_id, kInvalidTrackId) << "Could not build tracks.";

    // Update feature to track map.
    reconstruction->PutTrackToMap(added_track_id, track);
  }

  VLOG(1)
      << reconstruction->NumTracks() << " tracks were created. "
      << num_inconsistent_features
      << " features were dropped because they formed inconsistent tracks, and "
      << num_small_tracks << " features were dropped because they did not have "
                             "enough observations.";
}

void TrackBuilder::AddNewTracks(Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);

  std::vector<ViewId> reconstruction_view_ids = reconstruction->ViewIds();
  ViewId last_view_id = *std::max_element(reconstruction_view_ids.begin(),
                                          reconstruction_view_ids.end());

  // Build a reverse map mapping feature ids to ImageNameFeaturePairs.
  std::unordered_map<uint64_t, const std::pair<ViewId, Feature>*> id_to_feature;
  id_to_feature.reserve(features_.size());
  for (const auto& feature : features_) {
    InsertOrDie(&id_to_feature, feature.second, &feature.first);
  }

  // Extract all connected components.
  std::unordered_map<uint64_t, std::unordered_set<uint64_t> > components;
  connected_components_->Extract(&components);

  // Each connected component is a track. Add all tracks to the reconstruction.
  int num_small_tracks = 0;
  int num_inconsistent_features = 0;
  int num_new_tracks = 0;
  int num_updated_tracks = 0;
  int num_with_last = 0;
  int num_sharing_tracks = 0;

  for (const auto& component : components) {
    // Skip singleton tracks.
    if (component.second.size() < min_track_length_) {
      ++num_small_tracks;
      continue;
    }

    std::vector<std::pair<ViewId, Feature> > track;
    track.reserve(component.second.size());

    bool has_last_view_id = false;
    // Add all features in the connected component to the track.
    std::unordered_set<ViewId> view_ids;
    for (const auto& feature_id : component.second) {
      const auto& feature_to_add = *FindOrDie(id_to_feature, feature_id);

      // Do not add the feature if the track already contains a feature from the
      // same image.
      if (!InsertIfNotPresent(&view_ids, feature_to_add.first)) {
        ++num_inconsistent_features;
        continue;
      }
      has_last_view_id |= feature_to_add.first == last_view_id;
      track.emplace_back(feature_to_add);
    }

    if (has_last_view_id) {
      num_with_last++;
    }

    std::unordered_set<TrackId> track_ids =
        reconstruction->FindTrackInMap(track);

    TrackId leader_track_id;
    if (track_ids.empty()) {
      // No other tracks have common features. Add new track.
      leader_track_id = reconstruction->AddTrack(track);
      CHECK_NE(leader_track_id, kInvalidTrackId) << "Could not build tracks.";
      num_new_tracks++;
    } else if (track_ids.size() == 1) {
      // Features from this tracks are found only in one track.
      // Update it to have all features certainly.
      leader_track_id = *track_ids.begin();
      reconstruction->UpdateTrack(leader_track_id, track);
      num_updated_tracks++;
    } else {
      // Have several tracks that share these features.
      // They must be united.
      num_sharing_tracks += 1;
      leader_track_id = reconstruction->AddTrack(track);
      CHECK_NE(leader_track_id, kInvalidTrackId)
          << "Track can not been initialized.";

      Track* leader_track = reconstruction->MutableTrack(leader_track_id);

      // If there is a track that was estimated, set new track to estimated.
      // TODO(stoyanovd): Choose point initialization in old or recent tracks?
      for (auto track_id : track_ids) {
        CHECK_NE(track_id, kInvalidTrackId) << "Not-initialized Track !!!";

        const Track* old_track = reconstruction->Track(track_id);
        if (old_track == nullptr) {
          continue;
        }
        if (old_track->IsEstimated()) {
          *leader_track->MutablePoint() = old_track->Point();
          leader_track->SetEstimated(true);
          break;
        }
      }

      // Remove outdated tracks.
      for (auto track_id : track_ids) {
        reconstruction->RemoveTrack(track_id);
        // TODO(stoyanovd): Clean feature_set_to_track accordingly.
      }
    }

    // Add or rewrite features in track map.
    reconstruction->PutTrackToMap(leader_track_id, track);
  }

  VLOG(1)
      << reconstruction->NumTracks() << " totally in reconstruction. "
      << num_inconsistent_features
      << " features were dropped because they formed inconsistent tracks, and "
      << num_small_tracks << " features were dropped because they did not have "
                             "enough observations.";
  VLOG(1) << "Totally " << num_new_tracks + num_updated_tracks << " with view "
          << last_view_id << " was processed.";
  VLOG(1) << "New: " << num_new_tracks
          << " and updated: " << num_updated_tracks;
  VLOG(1) << "Tracks that observed from last View: " << num_with_last;
  VLOG(1) << "Tracks that united several old tracks with help of last View: "
          << num_sharing_tracks;
}

uint64_t TrackBuilder::FindOrInsert(
    const std::pair<ViewId, Feature>& image_feature) {
  const uint64_t* feature_id = FindOrNull(features_, image_feature);

  // If the feature is present, return the id.
  if (feature_id != nullptr) {
    return *feature_id;
  }

  // Otherwise, add the feature.
  const uint64_t new_feature_id = num_features_;
  InsertOrDieNoPrint(&features_, image_feature, new_feature_id);

  // Increment the number of features.
  ++num_features_;

  return new_feature_id;
}

}  // namespace theia

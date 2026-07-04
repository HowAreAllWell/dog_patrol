#include "assignment_application_plan.hpp"

#include <unordered_map>
#include <utility>

namespace vision_demo_host {
namespace {

AssignmentApplicationPlan::Application MakeApplication(
    const AssignmentApplicationPlan::TrackApplicationCandidate &candidate,
    const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> &debug_rows,
    const int frame_idx) {
  AssignmentApplicationPlan::Application application;
  application.track_idx = candidate.track_idx;
  application.raw_track_id = candidate.raw_track_id;
  application.semantic_id = candidate.semantic_id;

  for (const auto &row : debug_rows) {
    if (row.frame_idx != frame_idx || row.track_idx != candidate.track_idx ||
        row.semantic_id != candidate.semantic_id || !row.accepted) {
      continue;
    }

    if (row.stage == "inactive_recover_candidate") {
      application.assignment_cost = 1.0F - row.final_score;
    } else if (row.stage == "new_semantic") {
      application.assignment_cost = 0.0F;
    } else {
      application.assignment_cost = row.final_score;
    }
    application.assignment_margin = row.stage == "new_semantic" ? 1.0F : row.margin;
    application.accepted_stage = row.stage;
    application.found_accepted_row = true;
    break;
  }

  return application;
}

}  // namespace

AssignmentApplicationPlan::Result AssignmentApplicationPlan::Build(
    const std::vector<TrackApplicationCandidate> &candidates,
    const std::vector<AcceptedDebugRow> &debug_rows,
    const int frame_idx,
    const std::unordered_set<int> &occupied_semantic_ids,
    SemanticIdAllocator *semantic_id_allocator,
    const std::vector<int> &raw_map_track_order) {
  Result result;
  result.applications.reserve(candidates.size());
  std::unordered_set<int> occupied = occupied_semantic_ids;

  std::unordered_map<int, int> first_owner_by_semantic_id;
  for (const auto &candidate : candidates) {
    if (candidate.track_idx < 0 || candidate.raw_track_id <= 0 || candidate.semantic_id <= 0) {
      continue;
    }

    TrackApplicationCandidate resolved = candidate;
    const auto first_owner = first_owner_by_semantic_id.find(candidate.semantic_id);
    if (first_owner == first_owner_by_semantic_id.end()) {
      first_owner_by_semantic_id[candidate.semantic_id] = candidate.track_idx;
      occupied.insert(candidate.semantic_id);
    } else if (semantic_id_allocator != nullptr) {
      resolved.semantic_id = semantic_id_allocator->Allocate(occupied);
      occupied.insert(resolved.semantic_id);
    }

    auto application = MakeApplication(resolved, debug_rows, frame_idx);
    result.applications.push_back(std::move(application));
  }

  std::unordered_map<int, Application> application_by_track_idx;
  application_by_track_idx.reserve(result.applications.size());
  for (const auto &application : result.applications) {
    application_by_track_idx[application.track_idx] = application;
  }

  std::unordered_set<int> inserted_track_indices;
  inserted_track_indices.reserve(result.applications.size());
  const auto append_raw_mapping = [&](const Application &application) {
    result.next_raw_to_semantic_entries.push_back(RawMapping{application.raw_track_id,
                                                             application.semantic_id});
    result.next_raw_to_semantic[application.raw_track_id] = application.semantic_id;
    inserted_track_indices.insert(application.track_idx);
  };

  for (const int track_idx : raw_map_track_order) {
    const auto application_it = application_by_track_idx.find(track_idx);
    if (application_it == application_by_track_idx.end()) {
      continue;
    }
    append_raw_mapping(application_it->second);
  }

  for (const auto &application : result.applications) {
    if (inserted_track_indices.count(application.track_idx) > 0) {
      continue;
    }
    append_raw_mapping(application);
  }

  return result;
}

}  // namespace vision_demo_host

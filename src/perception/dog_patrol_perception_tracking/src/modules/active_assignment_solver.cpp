#include "active_assignment_solver.hpp"

#include <algorithm>
#include <cstddef>

namespace vision_demo_host {
namespace {

std::vector<std::vector<float>> BuildSquareCost(const std::vector<std::vector<float>> &cost) {
  const std::size_t rows = cost.size();
  const std::size_t cols = rows == 0 ? 0 : cost.front().size();
  const std::size_t n = std::max(rows, cols);
  std::vector<std::vector<float>> square(n, std::vector<float>(n, ActiveAssignmentSolver::kBigCost));
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < cols; ++c) {
      square[r][c] = cost[r][c];
    }
  }
  return square;
}

std::vector<int> HungarianSolve(const std::vector<std::vector<float>> &cost) {
  if (cost.empty() || cost.front().empty()) {
    return {};
  }

  auto a = BuildSquareCost(cost);
  const int n = static_cast<int>(a.size());
  std::vector<float> u(n + 1, 0.0F);
  std::vector<float> v(n + 1, 0.0F);
  std::vector<int> p(n + 1, 0);
  std::vector<int> way(n + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<float> minv(n + 1, ActiveAssignmentSolver::kBigCost);
    std::vector<char> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      float delta = ActiveAssignmentSolver::kBigCost;
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (used[j]) {
          continue;
        }
        const float cur = a[static_cast<std::size_t>(i0 - 1)][static_cast<std::size_t>(j - 1)] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] > 0) {
      assignment[static_cast<std::size_t>(p[j] - 1)] = j - 1;
    }
  }
  return assignment;
}

float ActiveMaxCost(const ActiveAssignmentSolver::CandidateInput &candidate,
                    const AssociationEvidence &association,
                    const ActiveAssignmentSolver::Config &config) {
  float max_cost = std::clamp(config.active_assign_max_cost, 0.0F, 1.0F);
  if (candidate.missing_frames > 0 && candidate.missing_frames <= std::max(1, config.max_missing_frames) &&
      association.passed_final_cost_gate && !association.stage.empty()) {
    max_cost = std::min(1.0F, max_cost + std::max(0.0F, config.min_assignment_margin));
  }
  return max_cost;
}

}  // namespace

ActiveAssignmentSolver::Result ActiveAssignmentSolver::Solve(const std::vector<TrackInput> &tracks,
                                                             const std::vector<CandidateInput> &candidates,
                                                             const std::vector<std::vector<float>> &cost,
                                                             const std::vector<std::vector<float>> &appearance_cost,
                                                             const Config &config) {
  if (tracks.empty() || candidates.empty()) {
    return {};
  }

  Result result;
  std::vector<int> assign = HungarianAssignment(cost);
  std::vector<bool> pairwise_appearance_override(tracks.size(), false);
  if (tracks.size() == 2 && candidates.size() == 2 && assign.size() >= 2 && assign[0] >= 0 && assign[1] >= 0 &&
      assign[0] != assign[1]) {
    const bool both_recently_missing =
        candidates[0].missing_frames > 0 && candidates[1].missing_frames > 0 &&
        candidates[0].missing_frames <= std::max(1, config.max_missing_frames) &&
        candidates[1].missing_frames <= std::max(1, config.max_missing_frames);
    const int alt0 = 1 - assign[0];
    const int alt1 = 1 - assign[1];
    if (both_recently_missing && alt0 >= 0 && alt0 < 2 && alt1 >= 0 && alt1 < 2 && alt0 != alt1 &&
        cost.size() >= 2 && cost[0].size() >= 2 && cost[1].size() >= 2 &&
        appearance_cost.size() >= 2 && appearance_cost[0].size() >= 2 && appearance_cost[1].size() >= 2) {
      const float selected_final = cost[0][static_cast<std::size_t>(assign[0])] +
                                   cost[1][static_cast<std::size_t>(assign[1])];
      const float alternate_final = cost[0][static_cast<std::size_t>(alt0)] +
                                    cost[1][static_cast<std::size_t>(alt1)];
      const float selected_app = appearance_cost[0][static_cast<std::size_t>(assign[0])] +
                                 appearance_cost[1][static_cast<std::size_t>(assign[1])];
      const float alternate_app = appearance_cost[0][static_cast<std::size_t>(alt0)] +
                                  appearance_cost[1][static_cast<std::size_t>(alt1)];
      const bool appearance_override =
          selected_final < kBigCost * 0.5F && alternate_final < kBigCost * 0.5F &&
          alternate_final <= selected_final + 0.08F && alternate_app + 0.035F <= selected_app;
      if (selected_final < kBigCost * 0.5F && alternate_final < kBigCost * 0.5F) {
        PairwiseDebug row;
        row.first_track_row = 0;
        row.second_track_row = 1;
        row.selected_first_col = assign[0];
        row.selected_second_col = assign[1];
        row.alternate_first_col = alt0;
        row.alternate_second_col = alt1;
        row.selected_final_cost = selected_final;
        row.alternate_final_cost = alternate_final;
        row.selected_app_cost = selected_app;
        row.alternate_app_cost = alternate_app;
        row.margin = alternate_final - selected_final;
        row.appearance_override = appearance_override;
        result.pairwise_debug_rows.push_back(row);
      }
    }
  }

  result.assignments.reserve(tracks.size());
  for (std::size_t r = 0; r < tracks.size(); ++r) {
    if (r >= assign.size()) {
      continue;
    }
    const int c = assign[r];
    if (c < 0 || c >= static_cast<int>(candidates.size()) || r >= cost.size() ||
        c >= static_cast<int>(cost[r].size())) {
      continue;
    }
    const float cst = cost[r][static_cast<std::size_t>(c)];
    if (cst >= kBigCost * 0.5F) {
      continue;
    }
    const float margin = AssignmentMargin(cost, r, c);
    const auto &candidate = candidates[static_cast<std::size_t>(c)];
    const float max_cost = ActiveMaxCost(candidate, tracks[r].association, config);
    bool accepted = true;
    std::string reject_reason;
    if (cst > max_cost) {
      accepted = false;
      reject_reason = "active_assign_max_cost_reject";
    } else if (margin < std::max(0.0F, config.min_assignment_margin)) {
      const bool pairwise_override =
          r < pairwise_appearance_override.size() && pairwise_appearance_override[r] &&
          cst <= std::min(1.0F, max_cost + std::max(0.0F, config.min_assignment_margin));
      const bool recently_missing =
          candidate.missing_frames > 0 && candidate.missing_frames <= std::max(1, config.max_missing_frames);
      const bool margin_cushion_ok =
          recently_missing && margin >= 0.5F * std::max(0.0F, config.min_assignment_margin) &&
          cst <= std::min(1.0F, max_cost + 0.05F);
      if (!margin_cushion_ok && !pairwise_override) {
        accepted = false;
        reject_reason = "assignment_margin_reject";
      }
    }
    result.assignments.push_back(Assignment{static_cast<int>(r),
                                            c,
                                            tracks[r].track_idx,
                                            candidate.semantic_id,
                                            1.0F - cst,
                                            cst,
                                            margin,
                                            accepted,
                                            reject_reason,
                                            r < pairwise_appearance_override.size() &&
                                                pairwise_appearance_override[r]});
  }
  return result;
}

std::vector<int> ActiveAssignmentSolver::HungarianAssignment(const std::vector<std::vector<float>> &cost) {
  return HungarianSolve(cost);
}

float ActiveAssignmentSolver::AssignmentMargin(const std::vector<std::vector<float>> &cost,
                                               const std::size_t row,
                                               const int selected_col) {
  if (row >= cost.size() || selected_col < 0 || selected_col >= static_cast<int>(cost[row].size())) {
    return 0.0F;
  }
  const float selected = cost[row][static_cast<std::size_t>(selected_col)];
  float second = kBigCost;
  for (std::size_t c = 0; c < cost[row].size(); ++c) {
    if (static_cast<int>(c) == selected_col) {
      continue;
    }
    second = std::min(second, cost[row][c]);
  }
  if (second >= kBigCost * 0.5F) {
    return std::max(0.0F, 1.0F - std::clamp(selected, 0.0F, 1.0F));
  }
  return std::max(0.0F, second - selected);
}

}  // namespace vision_demo_host

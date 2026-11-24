/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace wolf_wbid {

struct NamedJointReference {
  std::vector<std::size_t> indices;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::uint64_t generation{0};
  bool valid{false};
};

inline bool coversRequiredJoints(const NamedJointReference& reference,
                                 const std::vector<std::size_t>& required_indices) {
  return reference.valid && !required_indices.empty() &&
      std::all_of(required_indices.begin(), required_indices.end(), [&reference](std::size_t index) {
        return std::find(reference.indices.begin(), reference.indices.end(), index) != reference.indices.end();
      });
}

// Resolve names outside the control loop. Partial references deliberately leave
// joints absent from the message (for example an arm) at their existing target.
inline bool resolveNamedJointReference(const std::vector<std::string>& known_names,
                                      const std::vector<std::string>& names,
                                      const std::vector<double>& positions,
                                      const std::vector<double>& velocities,
                                      NamedJointReference& result) {
  if (names.empty() || names.size() != positions.size() ||
      (!velocities.empty() && names.size() != velocities.size()))
    return false;

  NamedJointReference candidate;
  candidate.indices.reserve(names.size());
  candidate.positions = positions;
  candidate.velocities = velocities.empty() ? std::vector<double>(names.size(), 0.0) : velocities;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto found = std::find(known_names.begin(), known_names.end(), names[i]);
    if (names[i].empty() || found == known_names.end() ||
        std::find(names.begin(), names.begin() + i, names[i]) != names.begin() + i ||
        !std::isfinite(candidate.positions[i]) || !std::isfinite(candidate.velocities[i]))
      return false;
    candidate.indices.push_back(static_cast<std::size_t>(found - known_names.begin()));
  }
  candidate.valid = true;
  result = std::move(candidate);
  return true;
}

}  // namespace wolf_wbid

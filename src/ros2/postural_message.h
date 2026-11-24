/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */
#pragma once

#include <Eigen/Core>
#include <wolf_msgs/msg/postural_task.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace wolf_wbid {
namespace detail {

// The task vectors contain floating-base rows; the message names only actuated
// joints. Use the same model-index mapping as incoming named references.
inline void fillPosturalJointFields(
    wolf_msgs::msg::PosturalTask& message,
    const std::vector<std::string>& names,
    const std::vector<std::size_t>& model_indices,
    const Eigen::VectorXd& position, const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& position_reference, const Eigen::VectorXd& velocity_reference,
    const Eigen::VectorXd& position_error, const Eigen::VectorXd& velocity_error)
{
  const auto count = names.size();
  message.name = names;
  message.position_actual.resize(count);
  message.position_reference.resize(count);
  message.velocity_actual.resize(count);
  message.velocity_reference.resize(count);
  message.position_error.resize(count);
  message.velocity_error.resize(count);

  for (std::size_t i = 0; i < count; ++i) {
    const auto index = static_cast<Eigen::Index>(model_indices[i]);
    message.position_actual[i] = position(index);
    message.position_reference[i] = position_reference(index);
    message.velocity_actual[i] = velocity(index);
    message.velocity_reference[i] = velocity_reference.size() == velocity.size()
        ? velocity_reference(index) : 0.0;
    message.position_error[i] = position_error(index);
    message.velocity_error[i] = velocity_error(index);
  }
}

}  // namespace detail
}  // namespace wolf_wbid

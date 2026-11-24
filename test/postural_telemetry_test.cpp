/* SPDX-License-Identifier: MIT */
#include "postural_message.h"

#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}

int main() {
  // Six floating-base rows followed by three actuated joints, including an arm.
  // The message order deliberately differs from the generalized vector order.
  const std::vector<std::string> names{"rear_knee", "arm_elbow", "front_hip"};
  const std::vector<std::size_t> indices{8, 6, 7};
  Eigen::VectorXd position(9), velocity(9), reference(9), reference_velocity(9);
  Eigen::VectorXd error(9), error_velocity(9);
  position << -101, -102, -103, -104, -105, -106, 10, 20, 30;
  velocity << -201, -202, -203, -204, -205, -206, 1, 2, 3;
  reference << -301, -302, -303, -304, -305, -306, 11, 22, 33;
  reference_velocity << -401, -402, -403, -404, -405, -406, 4, 6, 8;
  error = reference - position;
  error_velocity = reference_velocity - velocity;

  wolf_msgs::msg::PosturalTask message;
  message.header.frame_id = "Joints";
  message.cost = 42.0;
  wolf_wbid::detail::fillPosturalJointFields(message, names, indices, position, velocity,
      reference, reference_velocity, error, error_velocity);

  require(message.name == names, "Actuated joint names or arm entry were changed");
  require(message.position_actual == std::vector<double>({30, 10, 20}),
          "Named actual positions use the wrong generalized rows");
  require(message.position_reference == std::vector<double>({33, 11, 22}),
          "Named position references use the wrong generalized rows");
  require(message.velocity_actual == std::vector<double>({3, 1, 2}),
          "Named actual velocities use the wrong generalized rows");
  require(message.velocity_reference == std::vector<double>({8, 4, 6}),
          "Named velocity references use the wrong generalized rows");
  require(message.position_error == std::vector<double>({3, 1, 2}),
          "Named position errors use the wrong generalized rows");
  require(message.velocity_error == std::vector<double>({5, 3, 4}),
          "Named velocity errors use the wrong generalized rows");
  require(message.header.frame_id == "Joints" && message.cost == 42.0,
          "Joint field assembly changed message metadata");

  // Preserve the publisher's zero-reference-velocity fallback while still
  // emitting exactly the named actuated entries, without empty base labels.
  wolf_wbid::detail::fillPosturalJointFields(message, names, indices, position, velocity,
      reference, Eigen::VectorXd{}, error, error_velocity);
  require(message.velocity_reference == std::vector<double>({0, 0, 0}),
          "Missing reference velocities did not produce named zero entries");
  require(message.name.size() == 3 && message.position_actual.size() == 3 &&
          message.position_reference.size() == 3 && message.velocity_actual.size() == 3 &&
          message.velocity_reference.size() == 3 && message.position_error.size() == 3 &&
          message.velocity_error.size() == 3,
          "Telemetry arrays contain unnamed floating-base rows");
  std::cout << "Postural telemetry preserves named generalized-index mapping and arm entries\n";
}

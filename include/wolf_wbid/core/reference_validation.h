/* SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */
#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <string>

namespace wolf_wbid {

inline bool validCartesianReference(const Eigen::Vector3d& position,
                                    const Eigen::Quaterniond& orientation,
                                    const Eigen::Matrix<double, 6, 1>& twist) {
  const double squared_norm = orientation.squaredNorm();
  return position.allFinite() && orientation.coeffs().allFinite() && twist.allFinite() &&
         std::isfinite(squared_norm) && squared_norm > 1e-12;
}

inline bool validPointForceReference(const Eigen::Vector3d& force,
                                     const Eigen::Vector3d& torque,
                                     const std::string& frame) {
  return force.allFinite() && torque.allFinite() && (frame.empty() || frame == "world");
}

}  // namespace wolf_wbid

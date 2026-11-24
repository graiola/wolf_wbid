#include <wolf_wbid/core/reference_freshness.h>
#include <wolf_wbid/core/reference_validation.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  std::array<wolf_wbid::ReferenceFreshness, 10> references;
  const auto ready = [&references](std::int64_t now) {
    return std::all_of(references.begin(), references.end(), [now](const auto& reference) {
      return reference.fresh(now, 250);
    });
  };
  require(!ready(1000), "An unreceived command was treated as fresh");
  for (std::size_t i = 0; i < references.size() - 1; ++i) references[i].received(references[i].generation(), 1000);
  require(!ready(1100), "Nine channels or a contact heartbeat cannot replace the missing tenth reference");
  references.back().received(references.back().generation(), 1000);
  require(ready(1100), "Complete fresh set rejected");
  for (std::size_t i = 0; i < references.size() - 1; ++i) references[i].received(references[i].generation(), 1200);
  require(!ready(1251), "One stale channel was hidden by fresh remaining channels");
  references.back().received(references.back().generation(), 1250);
  require(ready(1251), "Recovered channel failed readiness");
  const auto previous_generation = references[3].generation();
  references[3].invalidate();
  require(!ready(1251), "A non-MPC replacement did not invalidate readiness");
  references[3].received(previous_generation, 1251);
  require(!ready(1251), "In-flight pre-reset callback renewed an invalid command");
  references[3].received(references[3].generation(), 1251);
  require(ready(1251), "A new generation could not restore readiness");
  require(!references.back().fresh(1200, 250), "Clock reversal accepted");

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  require(wolf_wbid::validCartesianReference(zero, Eigen::Quaterniond::Identity(), twist), "Valid Cartesian target rejected");
  require(!wolf_wbid::validCartesianReference(zero, Eigen::Quaterniond(0, 0, 0, 0), twist), "Zero quaternion accepted");
  auto invalid_position = zero;
  invalid_position.x() = std::numeric_limits<double>::quiet_NaN();
  require(!wolf_wbid::validCartesianReference(invalid_position, Eigen::Quaterniond::Identity(), twist), "NaN position accepted");
  twist(4) = std::numeric_limits<double>::infinity();
  require(!wolf_wbid::validCartesianReference(zero, Eigen::Quaterniond::Identity(), twist), "Infinite twist accepted");
  require(wolf_wbid::validPointForceReference(zero, zero, "world"), "World point force rejected");
  require(!wolf_wbid::validPointForceReference(zero, zero, "base"), "Local force silently interpreted in world");
  require(!wolf_wbid::validPointForceReference(invalid_position, zero, "world"), "NaN force accepted");
  std::cout << "Reference readiness and validation tests passed\n";
}

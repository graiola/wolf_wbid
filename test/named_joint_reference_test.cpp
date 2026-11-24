#include <wolf_wbid/core/named_joint_reference.h>

#include <iostream>
#include <limits>
#include <stdexcept>

static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  const std::vector<std::string> names{"lf_haa", "lf_hfe", "arm_joint"};
  wolf_wbid::NamedJointReference reference;
  require(wolf_wbid::resolveNamedJointReference(names, {"lf_hfe", "lf_haa"},
                                               {0.4, 0.2}, {0.3, 0.1}, reference),
          "Reordered partial reference rejected");
  std::vector<double> position{1.0, 2.0, 3.0}, velocity{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < reference.indices.size(); ++i) {
    position[reference.indices[i]] = reference.positions[i];
    velocity[reference.indices[i]] = reference.velocities[i];
  }
  require(position == std::vector<double>({0.2, 0.4, 3.0}), "Unnamed arm target was changed");
  require(velocity == std::vector<double>({0.1, 0.3, 0.0}), "Joint velocities were misordered");
  require(wolf_wbid::coversRequiredJoints(reference, {0, 1}),
          "Complete leg target was rejected because it omitted the arm");
  const auto saved = reference;
  require(!wolf_wbid::resolveNamedJointReference(names, {"lf_haa", "lf_haa"}, {1, 2}, {}, reference),
          "Duplicate named joints accepted");
  require(!wolf_wbid::resolveNamedJointReference(names, {"unknown"}, {1}, {}, reference),
          "Unknown joint accepted");
  require(!wolf_wbid::resolveNamedJointReference(names, {"lf_haa"}, {1, 2}, {}, reference),
          "Position dimension mismatch accepted");
  require(!wolf_wbid::resolveNamedJointReference(names, {"lf_haa"}, {1}, {1, 2}, reference),
          "Velocity dimension mismatch accepted");
  require(!wolf_wbid::resolveNamedJointReference(names, {"lf_haa"},
                                                {std::numeric_limits<double>::infinity()}, {}, reference),
          "Non-finite target accepted");
  require(reference.indices == saved.indices && reference.positions == saved.positions,
          "Invalid message changed accepted target");
  require(wolf_wbid::resolveNamedJointReference(names, {"lf_haa"}, {0.5}, {}, reference) &&
          reference.velocities == std::vector<double>{0.0}, "Missing velocities must default to zero");
  require(!wolf_wbid::coversRequiredJoints(reference, {0, 1}),
          "Partial leg target renewed the full MPC postural channel");
  std::cout << "Named joint reference tests passed\n";
}

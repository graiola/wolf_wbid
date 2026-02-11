#include <wolf_wbid/core/task_interface.h>

namespace wolf_wbid {

TaskWrapperInterface::TaskWrapperInterface(const std::string& task_name,
                                           const std::string& robot_name,
                                           const double& period)
: period_(period)
, task_name_(task_name)
, robot_name_(robot_name)
{
  // nothing else
}

void TaskWrapperInterface::update()
{
  // Deterministic order
  applyExternalKnobs();
  applyExternalReference();
  doUpdate();

  // note: publish/updateCost are intentionally NOT called here
  // The solver can call them explicitly when desired.
}

// --------------------
// DDR callbacks: only write buffers
// --------------------
void TaskWrapperInterface::setLambda1(double value)    { buffer_lambda1_.store(value); }
void TaskWrapperInterface::setLambda2(double value)    { buffer_lambda2_.store(value); }
void TaskWrapperInterface::setWeightDiag(double value) { buffer_weight_diag_.store(value); }

void TaskWrapperInterface::setKpX(double value)    { buffer_kp_x_.store(value); }
void TaskWrapperInterface::setKpY(double value)    { buffer_kp_y_.store(value); }
void TaskWrapperInterface::setKpZ(double value)    { buffer_kp_z_.store(value); }
void TaskWrapperInterface::setKpRoll(double value) { buffer_kp_roll_.store(value); }
void TaskWrapperInterface::setKpPitch(double value){ buffer_kp_pitch_.store(value); }
void TaskWrapperInterface::setKpYaw(double value)  { buffer_kp_yaw_.store(value); }

void TaskWrapperInterface::setKdX(double value)    { buffer_kd_x_.store(value); }
void TaskWrapperInterface::setKdY(double value)    { buffer_kd_y_.store(value); }
void TaskWrapperInterface::setKdZ(double value)    { buffer_kd_z_.store(value); }
void TaskWrapperInterface::setKdRoll(double value) { buffer_kd_roll_.store(value); }
void TaskWrapperInterface::setKdPitch(double value){ buffer_kd_pitch_.store(value); }
void TaskWrapperInterface::setKdYaw(double value)  { buffer_kd_yaw_.store(value); }

} // namespace wolf_wbid

#include <wolf_wbid/task_interface.h>

using namespace wolf_wbid;

TaskWrapperInterface::TaskWrapperInterface(const std::string& task_name, const std::string& robot_name, const double& period)
{
  assert(period > 0.0);
  period_ = period;
  last_time_ = 0.0;
  task_name_ = task_name;
  robot_name_ = robot_name;
}

TaskWrapperInterface::~TaskWrapperInterface()
{

}

void TaskWrapperInterface::setLambda1(double value)    {buffer_lambda1_ = value;     PRINT_INFO(task_name_<<" - "<<"Set lambda1: "<<value);}
void TaskWrapperInterface::setLambda2(double value)    {buffer_lambda2_ = value;     PRINT_INFO(task_name_<<" - "<<"Set lambda2: "<<value);}
void TaskWrapperInterface::setWeightDiag(double value) {buffer_weight_diag_ = value; PRINT_INFO(task_name_<<" - "<<"Set weight diagonal: "<<value);}

void TaskWrapperInterface::setKpX(double value)     { buffer_kp_x_     = value; PRINT_INFO(task_name_<<" - "<<"Set Kp(0,0): "<<value); }
void TaskWrapperInterface::setKpY(double value)     { buffer_kp_y_     = value; PRINT_INFO(task_name_<<" - "<<"Set Kp(1,1): "<<value); }
void TaskWrapperInterface::setKpZ(double value)     { buffer_kp_z_     = value; PRINT_INFO(task_name_<<" - "<<"Set Kp(2,2): "<<value); }
void TaskWrapperInterface::setKpRoll(double value)  { buffer_kp_roll_  = value; PRINT_INFO(task_name_<<" - "<<"Set Kp(3,3): "<<value); }
void TaskWrapperInterface::setKpPitch(double value) { buffer_kp_pitch_ = value; PRINT_INFO(task_name_<<" - "<<"Set Kp(4,4): "<<value); }
void TaskWrapperInterface::setKpYaw(double value)   { buffer_kp_yaw_   = value; PRINT_INFO(task_name_<<" - "<<"Set Kp(5,5): "<<value); }

void TaskWrapperInterface::setKdX(double value)     { buffer_kd_x_     = value; PRINT_INFO(task_name_<<" - "<<"Set Kd(0,0): "<<value); }
void TaskWrapperInterface::setKdY(double value)     { buffer_kd_y_     = value; PRINT_INFO(task_name_<<" - "<<"Set Kd(1,1): "<<value); }
void TaskWrapperInterface::setKdZ(double value)     { buffer_kd_z_     = value; PRINT_INFO(task_name_<<" - "<<"Set Kd(2,2): "<<value); }
void TaskWrapperInterface::setKdRoll(double value)  { buffer_kd_roll_  = value; PRINT_INFO(task_name_<<" - "<<"Set Kd(3,3): "<<value); }
void TaskWrapperInterface::setKdPitch(double value) { buffer_kd_pitch_ = value; PRINT_INFO(task_name_<<" - "<<"Set Kd(4,4): "<<value); }
void TaskWrapperInterface::setKdYaw(double value)   { buffer_kd_yaw_   = value; PRINT_INFO(task_name_<<" - "<<"Set Kd(5,5): "<<value); }

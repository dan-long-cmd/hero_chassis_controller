#include "hero_chassis_controller/use_pid_control.h"
#include <pluginlib/class_list_macros.hpp>
//setCommandCB

namespace hero_control_4 {

// 初始化：分配硬件句柄，加载PID参数
bool WheelController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
          ros::NodeHandle& n,
          const std::string& joint_name) {
  joint_ = effort_joint_interface->getHandle(joint_name);
  // 注意这里，直接把命名空间传给它自己
  if (!pid_.init(ros::NodeHandle(n, "pid/" + joint_name)))
  return false;
  controller_state_publisher_.reset(
  new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
  (n, joint_name+"/state", 1));
  return true;
}

 // 单个轮子的计算逻辑
void WheelController::update(const ros::Time& time, const ros::Duration& period) {
  double error = command_ - joint_.getVelocity();
  double effort = pid_.computeCommand(error, period);
  joint_.setCommand(effort);
  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = effort;
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}



  //对类的初始化->
ChassisController::ChassisController() {

}
ChassisController::~ChassisController()
{
  sub_command_.shutdown();
}

bool ChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::vector<std::string> joint_names;
  if (!n.getParam("joint_name", joint_names)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  left_front_.init(effort_joint_interface, n, joint_names[0]);
  right_front_.init(effort_joint_interface, n, joint_names[1]);
  left_back_.init(effort_joint_interface, n, joint_names[2]);
  right_back_.init(effort_joint_interface, n, joint_names[3]);
  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &ChassisController::setCommandCB, this);

  return true;
}
/*
void ChassisController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void ChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void ChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void ChassisController::printDebug()
{
  pid_controller_.printValues();
}

std::string ChassisController::getJointName()
{
  return joint_.getName();
}

// Set the joint velocity command
void ChassisController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void ChassisController::getCommand(double& cmd)
{
  cmd = command_;
}
*/
void ChassisController::starting(const ros::Time& time)
{
}

void ChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  left_front_.update(time, period);
  right_front_.update(time, period);
  left_back_.update(time, period);
  right_back_.update(time, period);
}

void ChassisController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  left_front_.command_ = msg->data;
  right_front_.command_ = msg->data;
  left_back_.command_ = msg->data;
  right_back_.command_ = msg->data;
}

} // namespace

PLUGINLIB_EXPORT_CLASS( hero_control_4::ChassisController, controller_interface::ControllerBase)
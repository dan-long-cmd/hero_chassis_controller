#pragma once
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

namespace effort_controllers
{
class JointVelocityController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  JointVelocityController();
  ~JointVelocityController();
  bool init(hardware_interface::EffortJointInterface *robot, const std::string &joint_name, const control_toolbox::Pid &pid);
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  void setCommand(double cmd);
  void getCommand(double & cmd);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
  void printDebug();
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
  std::string getJointName();
  hardware_interface::JointHandle joint_;
  double command_;                                /**< Last commanded velocity. */

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;           /**< Internal PID controller. */

  std::unique_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;
  ros::Subscriber sub_command_;
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);
};
} // namespace
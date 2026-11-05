#pragma once
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

namespace hero_control_4
{
class WheelController {
private:
  int loop_count_;
  std::unique_ptr<
  realtime_tools::RealtimePublisher<
    control_msgs::JointControllerState> > controller_state_publisher_ ;
public:
    hardware_interface::JointHandle joint_;
    control_toolbox::Pid pid_;
    double command_;
    // 初始化：分配硬件句柄，加载PID参数
    bool init(hardware_interface::EffortJointInterface* robot,ros::NodeHandle& n,const std::string& joint_name) ;
    // 单个轮子的计算逻辑
    void update(const ros::Time& time, const ros::Duration& period);
};


class ChassisController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ChassisController();
  ~ChassisController();

  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  //void setCommand(double cmd);
  //void getCommand(double & cmd);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  //void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  //void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
  //void printDebug();
  //void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
  std::string getJointName();
                               /**< Last commanded velocity. */
  double command_;
private:

  WheelController left_front_;
  WheelController right_front_;
  WheelController left_back_;
  WheelController right_back_;       /**< Internal PID controller. */

  ros::Subscriber sub_command_;
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);
};
} // namespace
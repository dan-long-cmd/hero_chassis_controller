#ifndef USE_PID_CONTROL_H
#define USE_PID_CONTROL_H


#pragma once
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


namespace hero_control_4
{
  //参数的定义
#define radius 0.07625
#define length_x 0.2
#define length_y 0.2
  //运动学函数的声明
void inverse_kinematics(double linear_x, double linear_y, double angular_z, double* command);
void forward_kinematics(double left_front_w, double right_front_w, double left_back_w, double right_back_w, double* value);


class WheelController {
private:
  int loop_count_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_ ;
  double command_;
public:
    hardware_interface::JointHandle joint_;//生成句柄joint，用于与xacro联系
    control_toolbox::Pid pid_;//引用control_toolbox的pid类，
    void setCommandCB(double command);//设置速度
    double get_velocity(void);//获取速度

    bool init(hardware_interface::EffortJointInterface* effort_joint_interface,ros::NodeHandle& n,const std::string& joint_name) ;
    void update(const ros::Time& time, const ros::Duration& period);

};


class ChassisController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ChassisController();
  ~ChassisController();

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  std::string getJointName();

  void get_velocity();//获取速度函数
private:
//创建轮子类下的四个轮子，并保护起来
  WheelController left_front_;
  WheelController right_front_;
  WheelController left_back_;
  WheelController right_back_;
  //下达命令
  double command_linear_x = 0.0;
  double command_linear_y = 0.0;
  double command_angular_z = 0.0;
//当前的速度
  double current_vx = 0.0;
  double current_vy = 0.0;
  double current_wz = 0.0;
//里程碑
  double sum_distance_x = 0.0;
  double sum_distance_y = 0.0;
  double sum_angular_z = 0.0;

  ros::Subscriber sub_command_;//订阅话题twist
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > pub_v;//发布'/odom'话题
  void setCommandCB(const geometry_msgs::Twist::ConstPtr& msg);//设置速度
  int loop_count_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};
} // namespace

#endif
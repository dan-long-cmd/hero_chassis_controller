#include "hero_chassis_controller/use_pid_control.h"
#include <pluginlib/class_list_macros.hpp>
//setCommandCB

namespace hero_control_4
{
//逆运动解算函数
  void inverse_kinematics(double linear_x, double linear_y, double angular_z, double* command)
  {
    command[0] = (linear_x - linear_y - (length_x + length_y) * angular_z )/radius;
    command[1] = (linear_x + linear_y + (length_x + length_y) * angular_z )/radius;
    command[2] = (linear_x + linear_y - (length_x + length_y) * angular_z )/radius;
    command[3] = (linear_x - linear_y + (length_x + length_y) * angular_z )/radius;
  }

//正运动解算函数
  void forward_kinematics(double left_front_w, double right_front_w, double left_back_w, double right_back_w, double* value)
  {

    value[0] = (left_front_w + right_front_w + left_back_w + right_back_w) * radius /4.0;
    value[1] = (-left_front_w + right_front_w + left_back_w - right_back_w) * radius /4.0;
    value[2] = (-left_front_w + right_front_w - left_back_w + right_back_w) * radius /(4.0*(length_x + length_y));
  }

//轮子类的初始化
bool WheelController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
          ros::NodeHandle& n,
          const std::string& joint_name)
{
    //得到句柄，硬件接口effort_joint_interface寻找对应的joint的参数
  joint_ = effort_joint_interface->getHandle(joint_name);
    //类里面pid初始化
  if (!pid_.init(ros::NodeHandle(n, "pid/" + joint_name)))
  return false;
//作为发送者，初始化
  controller_state_publisher_.reset(
  new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
  (n, joint_name+"/state", 10));
  return true;
}

 // 单个轮子的计算逻辑
void WheelController::update(const ros::Time& time, const ros::Duration& period)
{
    //轮子自己的pid算术
  double error = command_ - joint_.getVelocity();
  double effort = pid_.computeCommand(error, period);
  joint_.setCommand(effort);
    //发送自身的速度、误差、时间戳、目标速度、输出的力量
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
//设置速度
void WheelController::setCommandCB(double command)
{
  command_ = command;
}

//获取速度
double WheelController::get_velocity(void)
  {
    return (joint_.getVelocity());
  }




  //以下是对主类控制
ChassisController::ChassisController()
{

}
ChassisController::~ChassisController()
{
  sub_command_.shutdown();
}

bool ChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n)
{
  //设立一个容器
  std::vector<std::string> joint_names;
    //获取节点句柄的joint_name所有参数，并将这些参数塞给容器
  if (!n.getParam("joint_name", joint_names)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
    //将各个轮子初始化
  left_front_.init(effort_joint_interface, n, joint_names[0]);
  right_front_.init(effort_joint_interface, n, joint_names[1]);
  left_back_.init(effort_joint_interface, n, joint_names[2]);
  right_back_.init(effort_joint_interface, n, joint_names[3]);


    //订阅twist话题
  sub_command_  =  n.subscribe("/cmd_vel", 1, &ChassisController::setCommandCB, this);
//发布话题'/odom'的初始化
  pub_v.reset(
    new realtime_tools::RealtimePublisher<nav_msgs::Odometry>
    (n, "/odom", 10));

  return true;
}

void ChassisController::starting(const ros::Time& time)
{
}

void ChassisController::update(const ros::Time& time, const ros::Duration& period)
{
    //获取当前的速度
    get_velocity();
    //更新
  left_front_.update(time, period);
  right_front_.update(time, period);
  left_back_.update(time, period);
  right_back_.update(time, period);
    //里程碑
    double dt = period.toSec();
    sum_distance_x += (current_vx * cos(sum_angular_z) - current_vy * sin(sum_angular_z)) * dt;
    sum_distance_y += (current_vx * sin(sum_angular_z) + current_vy * cos(sum_angular_z)) * dt;
    sum_angular_z += current_wz * dt;
    //'/odom'和tf话题的发布(这些话题都是弄好的，不用自己设立msg)
  if(loop_count_ % 10 == 0) {
    if(pub_v && pub_v->trylock())
    {
      pub_v->msg_.header.stamp = time;
      pub_v->msg_.header.frame_id = "odom";
      pub_v->msg_.child_frame_id = "base_link";

      pub_v->msg_.pose.pose.position.x = sum_distance_x;
      pub_v->msg_.pose.pose.position.y = sum_distance_y;
      pub_v->msg_.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, sum_angular_z);
      pub_v->msg_.pose.pose.orientation.x = q.x();
      pub_v->msg_.pose.pose.orientation.y = q.y();
      pub_v->msg_.pose.pose.orientation.z = q.z();
      pub_v->msg_.pose.pose.orientation.w = q.w();


      pub_v->msg_.twist.twist.linear.x = current_vx;
      pub_v->msg_.twist.twist.linear.y = current_vy;
      pub_v->msg_.twist.twist.linear.z = 0.0;

      pub_v->msg_.twist.twist.angular.x = 0.0;
      pub_v->msg_.twist.twist.angular.y = 0.0;
      pub_v->msg_.twist.twist.angular.z = current_wz;
      pub_v->unlockAndPublish();

      geometry_msgs::TransformStamped t;
      t.header.stamp = time;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";
      t.transform.translation.x = sum_distance_x;
      t.transform.translation.y = sum_distance_y;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_broadcaster_.sendTransform(t);
    }
  }
  loop_count_++;
}
//回调函数：设置新的速度
void ChassisController::setCommandCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  double command[4] = {0};
    //逆运动解算
  inverse_kinematics(msg->linear.x, msg->linear.y, msg->angular.z, command);
    //将解算出来的值给到各个轮子
  left_front_.setCommandCB(command[0]);
  right_front_.setCommandCB(command[1]);
  left_back_.setCommandCB(command[2]);
  right_back_.setCommandCB(command[3]);
}
//获取速度
void ChassisController::get_velocity()
{
    double value[3] = {0};
//将轮子得到的速度正解算
    forward_kinematics(left_front_.get_velocity(), right_front_.get_velocity(), left_back_.get_velocity(), right_back_.get_velocity(), value);
//正解算得到的速度复制给current的
    current_vx = value[0];
    current_vy = value[1];
    current_wz = value[2];
}


} // namespace

PLUGINLIB_EXPORT_CLASS( hero_control_4::ChassisController, controller_interface::ControllerBase)
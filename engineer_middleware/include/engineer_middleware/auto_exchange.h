//
// Created by lsy on 23-7-15.
//
#pragma once
#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/ExchangerMsg.h>
#include <control_toolbox/pid.h>

namespace auto_exchange
{
enum FindProcess
{
  SWING,
  FOUND,
  LOCKED
};

enum AdjustProcess
{
  COMPUTER,
  MOVE_CHASSIS
};

enum ServoMoveProcess
{
  YZ,
  YAW,
  ROLL,
  Y,
  PITCH,
  REY,
  Z,
  PUSH,
  DONE
};

enum MotionMoveProcess
{
  SPHERE,
  LINE,
  POINT,
  ACHIEVE
};

enum ExchangeProcess
{
  FIND,
  PRE_ADJUST,
  MOVE,
  POST_ADJUST,
  FINISH
};

class JointInfo
{
public:
  JointInfo(XmlRpc::XmlRpcValue& joint)
  {
    offset_ = xmlRpcGetDouble(joint, "offset", 0.);
    max_scale_ = xmlRpcGetDouble(joint, "max_scale", 1.0);
    near_tolerance_ = xmlRpcGetDouble(joint, "near_tolerance", 0.05);
    ROS_ASSERT(joint["range"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    min_position_ = xmlRpcGetDouble(joint["range"], 0);
    max_position_ = xmlRpcGetDouble(joint["range"], 1);
    move_direct_ = -1;
  }
  bool judgeJointLimit()
  {
    return abs(current_position_ - offset_ - min_position_) <= near_tolerance_ ||
           abs(max_position_ + offset_ - current_position_) <= near_tolerance_;
  }
  void judgeMoveDirect()
  {
    if (current_position_ - offset_ <= min_position_)
      move_direct_ = 1;
    else if (current_position_ - offset_ >= max_position_)
      move_direct_ = -1;
  }

public:
  int move_direct_;
  double offset_, max_position_, min_position_, current_position_, max_scale_, near_tolerance_;
};

class ProgressBase
{
public:
  ProgressBase(XmlRpc::XmlRpcValue& progress, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : tf_buffer_(tf_buffer), nh_(nh)
  {
    time_out_ = xmlRpcGetDouble(progress, "timeout", 1e10);
  }
  virtual void init() = 0;
  virtual void nextProcess() = 0;
  virtual void manageProcess() = 0;
  virtual void run() = 0;
  virtual void printProcess() = 0;
  bool checkTimeout(ros::Duration period)
  {
    if (period.toSec() > time_out_)
    {
      ROS_ERROR("Step timeout,it should be finish in %f seconds", time_out_);
      return true;
    }
    return false;
  }

protected:
  tf2_ros::Buffer& tf_buffer_;
  int process_{};
  bool is_finish_{ false }, is_recorded_time_{ false };
  double time_out_{};
  ros::Time start_time_{};
  ros::NodeHandle nh_{};
};

class Find : public ProgressBase
{
public:
  Find(XmlRpc::XmlRpcValue& find, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh) : ProgressBase(find, tf_buffer, nh)
  {
    process_ = SWING;
    gimbal_scale_.resize(2, 0);
    chassis_scale_.resize(2, 0);
    search_range_ = xmlRpcGetDouble(find, "search_range", 0.3);
    ROS_ASSERT(find["yaw"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(find["pitch"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    yaw_ = new JointInfo(find["yaw"]);
    pitch_ = new JointInfo(find["pitch"]);
    confirm_lock_time_ = xmlRpcGetDouble(find, "confirm_lock_time", 10);
    visual_recognition_sub_ =
        nh_.subscribe<rm_msgs::ExchangerMsg>("/pnp_publisher", 10, &Find::visualRecognitionCallback, this);
  }
  void init() override
  {
    is_finish_ = false;
    is_recorded_time_ = false;
    process_ = { SWING };
    initScales();
  }
  void run() override
  {
    if (!is_finish_)
    {
      if (!is_recorded_time_)
      {
        is_recorded_time_ = true;
        start_time_ = ros::Time::now();
      }
      manageProcess();
      switch (process_)
      {
        case SWING:
        {
          autoSearch(false, true);
          if (checkTimeout(ros::Time::now() - start_time_))
          {
            is_finish_ = true;
            ROS_INFO_STREAM("TIME OUT");
          }
        }
        break;
        case FOUND:
        {
          for (int i = 0; i < (int)gimbal_scale_.size(); ++i)
          {
            gimbal_scale_[i] /= confirm_lock_time_;
          }
        }
        break;
        case LOCKED:
        {
          is_finish_ = true;
          ROS_INFO_STREAM("LOCKED");
        }
        break;
      }
    }
    else
    {
      initScales();
    }
  }
  std::vector<double> getGimbalScale()
  {
    return gimbal_scale_;
  }
  std::vector<double> getChassisScale()
  {
    return chassis_scale_;
  }

private:
  void nextProcess() override
  {
    process_++;
    printProcess();
  }
  void autoSearch(bool enable_chassis, bool enable_gimbal)
  {
    if (enable_gimbal)
    {
      geometry_msgs::TransformStamped base2yaw, yaw2pitch;
      base2yaw = tf_buffer_.lookupTransform("gimbal_base", "yaw", ros::Time(0));
      yaw2pitch = tf_buffer_.lookupTransform("yaw", "pitch", ros::Time(0));

      double yaw = yawFromQuat(base2yaw.transform.rotation);
      double roll_temp, pitch, yaw_temp;
      quatToRPY(yaw2pitch.transform.rotation, roll_temp, pitch, yaw_temp);
      yaw_->current_position_ = yaw / search_range_;
      pitch_->current_position_ = pitch / search_range_;
      yaw_->judgeMoveDirect();
      pitch_->judgeMoveDirect();
      gimbal_scale_[0] = yaw_->move_direct_ * yaw_->max_scale_;
      gimbal_scale_[1] = pitch_->move_direct_ * pitch_->max_scale_;
    }
    if (enable_chassis)
    {
      chassis_scale_[0] = gimbal_scale_[0];
      chassis_scale_[1] = gimbal_scale_[1];
    }
  }
  void manageProcess() override
  {
    if (!is_found_)
    {
      process_ = SWING;
    }
    else
    {
      static int lock_time = 0;
      if (lock_time <= confirm_lock_time_)
      {
        process_ = FOUND;
        lock_time++;
      }
      else
      {
        lock_time = 0;
        process_ = LOCKED;
        is_finish_ = true;
      }
    }
  }
  void printProcess() override
  {
    if (process_ == SWING)
      ROS_INFO_STREAM("SWING");
    else if (process_ == FOUND)
      ROS_INFO_STREAM("FOUND");
    else if (process_ == LOCKED)
      ROS_INFO_STREAM("LOCKED");
  }
  void initScales()
  {
    for (int i = 0; i < (int)gimbal_scale_.size(); ++i)
    {
      gimbal_scale_[i] = 0;
      chassis_scale_[i] = 0;
    }
  }
  void visualRecognitionCallback(const rm_msgs::ExchangerMsg ::ConstPtr& msg)
  {
    is_found_ = msg->flag;
  }
  JointInfo *yaw_{}, *pitch_{};
  bool is_found_{ false };
  std::vector<double> gimbal_scale_{}, chassis_scale_{};
  ros::Subscriber visual_recognition_sub_{};
  double search_range_{}, confirm_lock_time_{};
};

class ProAdjust : public ProgressBase
{
public:
  ProAdjust(XmlRpc::XmlRpcValue& pre_adjust, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(pre_adjust, tf_buffer, nh)
  {
    ROS_ASSERT(pre_adjust["chassis_p"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pre_adjust["chassis_exchanger_offset"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pre_adjust["chassis_start_vel"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    chassis_p_.resize(3, 0);
    chassis_start_vel_.resize(3, 0.05);
    chassis_exchanger_offset_.resize(3, 0);
    chassis_xy_tolerance_.resize(2, 1e10);
    for (int i = 0; i < (int)chassis_exchanger_offset_.size(); ++i)
    {
      chassis_p_[i] = pre_adjust["chassis_p"][i];
      chassis_start_vel_[i] = pre_adjust["chassis_start_vel"][i];
      chassis_exchanger_offset_[i] = pre_adjust["chassis_exchanger_offset_"][i];
    }
    ROS_INFO_STREAM("~~~~~~~~~~~~~PRE_ADJUST~~~~~~~~~~~~~~~~");
  }
  void init() override
  {
    is_finish_ = false;
    is_recorded_time_ = false;
    enter_pre_adjust_ = false;
  }
  void run() override
  {
    enter_pre_adjust_ = true;
    if (!is_recorded_time_)
    {
      start_time_ = ros::Time::now();
      setChassisTarget(chassis_exchanger_offset_[0], chassis_exchanger_offset_[1], chassis_exchanger_offset_[2]);
    }
    if (!is_finish_)
    {
      computerChassisVel();
    }
    else if (checkTimeout(ros::Time::now() - start_time_))
    {
      is_finish_ = true;
    }
    else if (isChassisFinish())
    {
      is_finish_ = true;
      ROS_INFO_STREAM("CHASSIS ARRIVED");
    }
  }
  geometry_msgs::Twist getChassisVelMsg()
  {
    return chassis_vel_cmd_;
  }
  std::string getChassisCmdFrame()
  {
    return chassis_command_source_frame_;
  }

private:
  void initComputerValue()
  {
    chassis_vel_cmd_.linear.x = 0;
    chassis_vel_cmd_.linear.y = 0;
    chassis_vel_cmd_.linear.z = 0;
    chassis_vel_cmd_.angular.x = 0;
    chassis_vel_cmd_.angular.y = 0;
    chassis_vel_cmd_.angular.z = 0;
  }
  void nextProcess() override{};
  void manageProcess() override{};
  void printProcess() override
  {
    ROS_INFO_STREAM("PRE ADJUST");
  }
  bool isChassisFinish()
  {
    return (((chassis_pos_error_[0] <= chassis_xy_tolerance_[0]) &&
             (chassis_pos_error_[1] <= chassis_xy_tolerance_[1]) && (chassis_yaw_error_ <= chassis_yaw_tolerance_)));
  }
  void computerChassisVel()
  {
    geometry_msgs::TransformStamped current;
    current = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
    geometry_msgs::Vector3 error;
    error.x = chassis_target_.pose.position.x - current.transform.translation.x;
    error.y = chassis_target_.pose.position.y - current.transform.translation.y;

    double roll, pitch, yaw_current, yaw_goal, error_yaw;
    quatToRPY(current.transform.rotation, roll, pitch, yaw_current);
    quatToRPY(chassis_target_.pose.orientation, roll, pitch, yaw_goal);
    error_yaw = angles::shortest_angular_distance(yaw_current, yaw_goal);

    ROS_INFO_STREAM(error.x);
    ROS_INFO_STREAM(error.y);
    ROS_INFO_STREAM(error_yaw);

    chassis_vel_cmd_.linear.x = (error.x / abs(error.x)) * (chassis_start_vel_[0] + chassis_p_[0] * abs(error.x));
    chassis_vel_cmd_.linear.y = (error.y / abs(error.y)) * (chassis_start_vel_[1] + chassis_p_[1] * abs(error.y));
    chassis_vel_cmd_.angular.z =
        (error_yaw / abs(error_yaw)) * (chassis_start_vel_[2] + chassis_p_[2] * abs(error_yaw));
  }
  void setChassisTarget(double target_x, double target_y, double yaw_p)
  {
    geometry_msgs::TransformStamped exchange2base;
    double roll, pitch, yaw;
    exchange2base = tf_buffer_.lookupTransform("base_link", "exchanger", ros::Time(0));
    quatToRPY(exchange2base.transform.rotation, roll, pitch, yaw);
    //        ROS_INFO_STREAM(yaw);
    double goal_x = exchange2base.transform.translation.x - target_x;
    double goal_y = exchange2base.transform.translation.y - target_y;
    double goal_yaw = yaw * yaw_p;
    chassis_original_target_.pose.position.x = goal_x;
    chassis_original_target_.pose.position.y = goal_y;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, goal_yaw);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    chassis_target_.pose.orientation = quat_msg;
    chassis_target_ = chassis_original_target_;

    tf2::doTransform(chassis_target_, chassis_target_, tf_buffer_.lookupTransform("base_link", "map", ros::Time(0)));
    chassis_pos_error_[0] = 1e10;
    chassis_pos_error_[1] = 1e10;
    chassis_yaw_error_ = 1e10;
  }
  bool enter_pre_adjust_{ false };
  std::string chassis_command_source_frame_{ "base_link" };
  std::vector<double> chassis_exchanger_offset_, chassis_xy_tolerance_, chassis_pos_error_, chassis_start_vel_,
      chassis_p_;
  geometry_msgs::Twist chassis_vel_cmd_{};
  geometry_msgs::PoseStamped chassis_target_{}, chassis_original_target_{};
  double chassis_yaw_tolerance_{}, chassis_yaw_error_{};
};

class AutoServoMove : public ProgressBase
{
public:
  AutoServoMove(XmlRpc::XmlRpcValue& auto_servo_move, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(auto_servo_move, tf_buffer, nh), joint7_msg_(0.)
  {
    ROS_ASSERT(auto_servo_move["xyz_offset"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["servo_error_tolerance"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["link7_length"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    process_ = { YZ };
    enter_auto_servo_move_.data = false;
    xyz_offset_.resize(3, 0.);
    servo_pid_value_.resize(6, 0.);
    servo_errors_.resize(6, 0.);
    servo_scales_.resize(6, 0.);
    servo_error_tolerance_.resize(6, 0.01);
    ros::NodeHandle nh_servo_pid = ros::NodeHandle(nh, "servo_pid");
    ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_servo_pid, "x");
    ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_servo_pid, "y");
    ros::NodeHandle nh_pid_z = ros::NodeHandle(nh_servo_pid, "z");
    ros::NodeHandle nh_pid_roll = ros::NodeHandle(nh_servo_pid, "roll");
    ros::NodeHandle nh_pid_pitch = ros::NodeHandle(nh_servo_pid, "pitch");
    ros::NodeHandle nh_pid_yaw = ros::NodeHandle(nh_servo_pid, "yaw");
    pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
    pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
    pid_z_.init(ros::NodeHandle(nh_pid_z, "pid"));
    pid_roll_.init(ros::NodeHandle(nh_pid_roll, "pid"));
    pid_pitch_.init(ros::NodeHandle(nh_pid_pitch, "pid"));
    pid_yaw_.init(ros::NodeHandle(nh_pid_yaw, "pid"));

    for (int i = 0; i < (int)xyz_offset_.size(); ++i)
      xyz_offset_[i] = auto_servo_move["xyz_offset"][i];
    for (int i = 0; i < (int)servo_error_tolerance_.size(); ++i)
    {
      servo_error_tolerance_[i] = auto_servo_move["servo_error_tolerance"][i];
    }
    exchanger_tf_update_pub_ = nh_.advertise<std_msgs::Bool>("/is_update_exchanger", 1);
    ROS_INFO_STREAM("~~~~~~~~~~~~~SERVO_MOVE~~~~~~~~~~~~~~~~");
  }
  ~AutoServoMove() = default;
  void init() override
  {
    is_finish_ = false;
    is_recorded_time_ = false;
    process_ = { YZ };
    enter_auto_servo_move_.data = false;
    is_exchanger_tf_update_.data = true;
    initComputerValue();
    joint7_msg_ = 0.;
    exchanger_tf_update_pub_.publish(is_exchanger_tf_update_);
  }
  void printProcess() override
  {
    if (process_ == YZ)
      ROS_INFO_STREAM("YZ");
    else if (process_ == YAW)
      ROS_INFO_STREAM("YAW");
    else if (process_ == ROLL)
      ROS_INFO_STREAM("ROLL");
    else if (process_ == Y)
      ROS_INFO_STREAM("Y");
    else if (process_ == PITCH)
      ROS_INFO_STREAM("PITCH");
    else if (process_ == REY)
      ROS_INFO_STREAM("REY");
    else if (process_ == Z)
      ROS_INFO_STREAM("Z");
    else if (process_ == PUSH)
      ROS_INFO_STREAM("PUSH");
    else if (process_ == DONE)
      ROS_INFO_STREAM("DOWN");
  }
  void run() override
  {
    is_exchanger_tf_update_.data = false;
    exchanger_tf_update_pub_.publish(is_exchanger_tf_update_);
    enter_auto_servo_move_.data = true;
    if (!is_finish_)
    {
      is_enter_auto_ = true;
      if (!is_recorded_time_)
      {
        start_time_ = ros::Time::now();
        is_recorded_time_ = true;
      }
      computeServoMoveScale();
      manageProcess();
    }
  }
  double getJoint7Msg()
  {
    return joint7_msg_;
  }
  bool getEnterAutoServoFlag()
  {
    return enter_auto_servo_move_.data;
  }
  bool getFinishFlag()
  {
    return is_finish_;
  }
  std::vector<double> getServoScale()
  {
    return servo_scales_;
  }

private:
  void nextProcess() override
  {
    process_++;
    printProcess();
    is_recorded_time_ = false;
  }
  void manageProcess() override
  {
    manageServoMoveProcess();
  }
  void initComputerValue()
  {
    for (int i = 0; i < (int)servo_scales_.size(); ++i)
    {
      servo_errors_[i] = 0;
      servo_pid_value_[i] = 0;
      servo_scales_[i] = 0;
    }
  }
  void computeServoPidValue()
  {
    ros::Duration t(0.01);
    servo_pid_value_[0] = pid_x_.computeCommand(servo_errors_[0], t);
    servo_pid_value_[1] = pid_y_.computeCommand(servo_errors_[1], t);
    servo_pid_value_[2] = pid_z_.computeCommand(servo_errors_[2], t);
    servo_pid_value_[3] = pid_roll_.computeCommand(servo_errors_[3], t);
    servo_pid_value_[4] = pid_pitch_.computeCommand(servo_errors_[4], t);
    servo_pid_value_[5] = pid_yaw_.computeCommand(servo_errors_[5], t);
  }

  void computeServoMoveError()
  {
    initComputerValue();
    double roll, pitch, yaw, roll_base, pitch_base, yaw_base;
    std::vector<double> errors;
    geometry_msgs::TransformStamped tools2exchanger, base2exchanger;
    try
    {
      tools2exchanger = tf_buffer_.lookupTransform("tools_link", "exchanger", ros::Time(0));
      base2exchanger = tf_buffer_.lookupTransform("base_link", "exchanger", ros::Time(0));
      quatToRPY(tools2exchanger.transform.rotation, roll, pitch, yaw);
      quatToRPY(base2exchanger.transform.rotation, roll_base, pitch_base, yaw_base);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    initComputerValue();
    servo_errors_[0] = (process_ == PUSH ? (tools2exchanger.transform.translation.x) :
                                           (tools2exchanger.transform.translation.x - xyz_offset_[0]));
    servo_errors_[1] = tools2exchanger.transform.translation.y - xyz_offset_[1];
    servo_errors_[2] = tools2exchanger.transform.translation.z - xyz_offset_[2];
    servo_errors_[3] = roll;
    servo_errors_[4] = pitch;
    servo_errors_[5] = yaw;
  }
  void computeServoMoveScale()
  {
    computeServoMoveError();
    computeServoPidValue();
    switch (process_)
    {
      case YZ:
      {
        servo_scales_[1] = servo_errors_[1] * servo_pid_value_[1];
        servo_scales_[2] = servo_errors_[2] * servo_pid_value_[2];
      }
      break;
      case YAW:
      {
        servo_scales_[5] = servo_errors_[5] * servo_pid_value_[5];
      }
      break;
      case ROLL:
      {
        servo_scales_[3] = servo_errors_[3] * servo_pid_value_[3];
      }
      break;
      case Y:
      {
        servo_scales_[1] = servo_errors_[1] * servo_pid_value_[1];
      }
      break;
      case PITCH:
      {
        joint7_msg_ = servo_errors_[4];
      }
      break;
      case REY:
      {
        servo_scales_[1] = servo_errors_[1] * servo_pid_value_[1];
      }
      break;
      case Z:
      {
        servo_scales_[2] = servo_errors_[2] * servo_pid_value_[2];
      }
      break;
      case PUSH:
      {
        servo_scales_[0] = servo_errors_[0] * servo_pid_value_[0];
      }
      break;
    }
  }
  void manageServoMoveProcess()
  {
    int move_joint_num = 0, arrived_joint_num = 0;
    for (int i = 0; i < (int)servo_scales_.size(); ++i)
    {
      if (servo_scales_[i] != 0)
      {
        move_joint_num++;
        if (abs(servo_errors_[i]) <= servo_error_tolerance_[i])
          arrived_joint_num++;
      }
    }
    if (checkTimeout(ros::Time::now() - start_time_))
    {
      if (process_ != DONE)
        nextProcess();
      else
        is_finish_ = true;
      ROS_INFO_STREAM("TIME OUT");
    }
    else if (arrived_joint_num == move_joint_num)
    {
      if (process_ != DONE)
        nextProcess();
      else
        is_finish_ = true;
    }
  }

  ros::Time inside_process_start_time_{};
  std_msgs::Bool enter_auto_servo_move_{}, is_exchanger_tf_update_{};
  bool is_enter_auto_{};
  double link7_length_{}, joint7_msg_{};
  ros::Publisher exchanger_tf_update_pub_;
  control_toolbox::Pid pid_x_, pid_y_, pid_z_, pid_roll_, pid_pitch_, pid_yaw_;
  std::vector<double> xyz_offset_{}, servo_errors_{}, servo_scales_{}, servo_error_tolerance_{}, servo_pid_value_{};
};

// class MotionMove
//{
//};
//
// class PostAdjust
//{
//};
//
// class Finish
//{
//};

}  // namespace auto_exchange

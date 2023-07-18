//
// Created by lsy on 23-7-15.
//
#pragma once
#include <rm_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/ExchangerMsg.h>

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
  bool is_finish_{ false };
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
    ROS_INFO_STREAM(time_out_);
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
  bool is_found_{ false }, is_recorded_time_{ false };
  std::vector<double> gimbal_scale_{}, chassis_scale_{};
  ros::Subscriber visual_recognition_sub_{};
  double search_range_{}, confirm_lock_time_{};
};

// class ProAdjust
//{
//};

class AutoServoMove : public ProgressBase
{
public:
  AutoServoMove(XmlRpc::XmlRpcValue& auto_servo_move, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(auto_servo_move, tf_buffer, nh), joint7_msg_(0.)
  {
    ROS_ASSERT(auto_servo_move["xyz_offset"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["servo_p"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["servo_error_tolerance"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["link7_length"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    process_ = { YZ };
    enter_auto_servo_move_.data = false;
    xyz_offset_.resize(3, 0.);
    servo_p_.resize(6, 0.);
    servo_errors_.resize(6, 0.);
    servo_scales_.resize(6, 0.);
    servo_error_tolerance_.resize(6, 0.01);
    for (int i = 0; i < (int)xyz_offset_.size(); ++i)
      xyz_offset_[i] = auto_servo_move["xyz_offset"][i];
    for (int i = 0; i < (int)servo_p_.size(); ++i)
    {
      servo_p_[i] = auto_servo_move["servo_p"][i];
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
    else if (process_ == REY)
      ROS_INFO_STREAM("REY");
    else if (process_ == PITCH)
      ROS_INFO_STREAM("PITCH");
    else if (process_ == Z)
      ROS_INFO_STREAM("Z");
    else if (process_ == PUSH)
      ROS_INFO_STREAM("PUSH");
    else if (process_ == DONE)
      ROS_INFO_STREAM("DOWN");
  }
  void run() override
  {
    enter_auto_servo_move_.data = true;
    is_exchanger_tf_update_.data = false;
    exchanger_tf_update_pub_.publish(is_exchanger_tf_update_);
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
      servo_scales_[i] = 0;
    }
  }
  void computeServoMoveError()
  {
    initComputerValue();
    double roll, pitch, yaw;
    std::vector<double> errors;
    geometry_msgs::TransformStamped tools2exchanger;
    try
    {
      tools2exchanger = tf_buffer_.lookupTransform("tools_link", "exchanger", ros::Time(0));
      quatToRPY(tools2exchanger.transform.rotation, roll, pitch, yaw);
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
    switch (process_)
    {
      case YZ:
      {
        for (int i = 1; i < 3; ++i)
        {
          servo_scales_[i] = servo_errors_[i] * servo_p_[i];
        }
      }
      break;
      case YAW:
      {
        servo_scales_[5] = servo_errors_[5] * servo_p_[5];
      }
      break;
      case ROLL:
      {
        servo_scales_[3] = servo_errors_[3] * servo_p_[3];
      }
      break;
      case Y:
      {
        servo_scales_[1] = servo_errors_[1] * servo_p_[1];
      }
      break;
      case PITCH:
      {
        joint7_msg_ = servo_errors_[4];
      }
      break;
      case REY:
      {
        servo_scales_[1] = servo_errors_[1] * servo_p_[1];
      }
      break;
      case Z:
      {
        servo_scales_[2] = servo_errors_[2] * servo_p_[2];
      }
      break;
      case PUSH:
      {
        servo_scales_[0] = servo_errors_[0] * servo_p_[0];
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
  bool is_recorded_time_{}, is_enter_auto_{};
  double link7_length_{}, joint7_msg_{};
  ros::Publisher exchanger_tf_update_pub_;
  std::vector<double> xyz_offset_{}, servo_p_{}, servo_errors_{}, servo_scales_{}, servo_error_tolerance_{};
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

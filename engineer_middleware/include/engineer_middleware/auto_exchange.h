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
#include <angles/angles.h>

namespace auto_exchange
{
enum MotionMoveProcess
{
  SPHERE,
  LINE,
  POINT,
  ACHIEVE
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
    time_out_ = xmlRpcGetDouble(progress, "time_out", 1e10);
    if (nh_.hasParam("internal_time_out"))
    {
      internal_time_out_.resize((int)progress["internal_time_out"].size(), 1.);
      for (int i = 0; i < (int)progress["internal_time_out"].size(); ++i)
        internal_time_out_[i] = (progress["internal_time_out"][i]);
    }
  }
  virtual void init()
  {
    enter_flag_ = false;
    is_finish_ = false;
    is_recorded_time_ = false;
    is_recorded_internal_time_ = false;
  }
  virtual void stateMachine() = 0;
  virtual void run()
  {
    enter_flag_ = true;
    if (!is_finish_)
    {
      checkTimeout();
      if (nh_.hasParam("internal_time_out"))
      {
        checkInternalTimeout();
      }
      stateMachine();
    }
  }
  virtual void printProcess() = 0;
  bool getFinishFlag()
  {
    return is_finish_;
  }
  bool getEnterFlag()
  {
    return enter_flag_;
  }
  void checkTimeout()
  {
    if (!is_recorded_time_)
    {
      is_recorded_time_ = true;
      start_time_ = ros::Time::now();
    }
    if ((ros::Time::now() - start_time_).toSec() > time_out_)
    {
      ROS_ERROR("Progress timeout, should be finish in %f seconds", time_out_);
      is_recorded_time_ = false;
      is_finish_ = true;
    }
  }
  void checkInternalTimeout()
  {
    if (!is_recorded_internal_time_ || last_process_ != process_)
    {
      is_recorded_internal_time_ = true;
      internal_start_time_ = ros::Time::now();
      last_process_ = process_;
    }
    if ((ros::Time::now() - internal_start_time_).toSec() > internal_time_out_[process_])
    {
      ROS_ERROR("Inside progress timeout, should be finish in %f seconds", internal_time_out_[process_]);
      //      ROS_INFO_STREAM("dt:  " <<  (ros::Time::now() - internal_start_time_).toSec());
      is_recorded_internal_time_ = false;
      if (process_ < (process_num_ - 1))
        process_++;
      else
        is_finish_ = true;
    }
  }

protected:
  tf2_ros::Buffer& tf_buffer_;
  int process_{}, last_process_{}, process_num_{};
  bool is_finish_{ false }, is_recorded_time_{ false }, enter_flag_{ false }, is_recorded_internal_time_{ false };
  double time_out_{};
  std::vector<double> internal_time_out_{};
  ros::Time start_time_{}, internal_start_time_{};
  ros::NodeHandle nh_{};
};

class Find : public ProgressBase
{
public:
  enum FindProcess
  {
    SWING,
    ADJUST,
    FINISH
  };
  Find(XmlRpc::XmlRpcValue& find, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh) : ProgressBase(find, tf_buffer, nh)
  {
    process_ = SWING;
    last_process_ = process_;
    process_num_ = 3;
    gimbal_scale_.resize(2, 0);
    chassis_scale_.resize(2, 0);
    search_range_ = xmlRpcGetDouble(find, "search_range", 0.3);
    ROS_ASSERT(find["yaw"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(find["pitch"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    yaw_ = new JointInfo(find["yaw"]);
    pitch_ = new JointInfo(find["pitch"]);
    visual_recognition_sub_ =
        nh_.subscribe<rm_msgs::ExchangerMsg>("/pnp_publisher", 1, &Find::visualRecognitionCallback, this);
    ROS_INFO_STREAM("~~~~~~~~~~~~~FIND~~~~~~~~~~~~~~~~");
  }
  void init() override
  {
    ProgressBase::init();
    process_ = { SWING };
    initScales();
  }
  std::vector<double> getGimbalScale()
  {
    return gimbal_scale_;
  }
  std::vector<double> getChassisScale()
  {
    return chassis_scale_;
  }
  void testAdjust()
  {
    gimbal_scale_[0] = middle_point_.x / 200;
    gimbal_scale_[1] = middle_point_.y / 200;
  }

private:
  void stateMachine() override
  {
    switch (process_)
    {
      case SWING:
      {
        autoSearch(false, true);
        if (is_found_)
          process_ = ADJUST;
      }
      break;
      case ADJUST:
      {
        if (is_found_)
        {
          gimbal_scale_[0] = middle_point_.x / abs(middle_point_.x) * yaw_->max_scale_;
          gimbal_scale_[1] = middle_point_.y / abs(middle_point_.y) * pitch_->max_scale_;
          if (sqrt(pow(middle_point_.x, 2) + pow(middle_point_.y, 2)) < 200)
            process_ = FINISH;
        }
        else
          process_ = SWING;
      }
      break;
      case FINISH:
      {
        is_finish_ = true;
        ROS_INFO_STREAM("LOCKED");
      }
      break;
    }
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
  void printProcess() override
  {
    if (process_ == SWING)
      ROS_INFO_STREAM("SWING");
    else if (process_ == ADJUST)
      ROS_INFO_STREAM("FOUND");
    else if (process_ == FINISH)
      ROS_INFO_STREAM("FINISH");
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
    middle_point_ = msg->middle_point;
  }
  JointInfo *yaw_{}, *pitch_{};
  bool is_found_{ false };
  geometry_msgs::Point middle_point_;
  std::vector<double> gimbal_scale_{}, chassis_scale_{};
  ros::Subscriber visual_recognition_sub_{};
  double search_range_{};
};

class ProAdjust : public ProgressBase
{
public:
  enum AdjustProcess
  {
    SET_GOAL,
    CHASSIS_Y,
    CHASSIS_X,
    CHASSIS_YAW,
    FINISH
  };
  struct ChassisSingleDirectionMove
  {
    std::string name;
    double tolerance, start_vel, offset_refer_exchanger, max_vel, error;
    control_toolbox::Pid pid;
    void init(XmlRpc::XmlRpcValue& chassis_move_config, std::string config_name, ros::NodeHandle& nh)
    {
      name = config_name;
      error = 1e10;
      max_vel = chassis_move_config["max_vel"];
      start_vel = chassis_move_config["start_vel"];
      tolerance = chassis_move_config["tolerance"];
      offset_refer_exchanger = chassis_move_config["offset_refer_exchanger"];
      ros::NodeHandle nh_chassis_move_config = ros::NodeHandle(nh, name);
      pid.init(ros::NodeHandle(nh_chassis_move_config, "pid"));
    }
    bool isFinish()
    {
      return abs(error) <= tolerance;
    }
    double computerVel(ros::Duration dt)
    {
      double vel = start_vel + abs(pid.computeCommand(error, dt));
      int direction = error / abs(error);
      return abs(vel) >= max_vel ? direction * max_vel : direction * vel;
    }
  };
  ProAdjust(XmlRpc::XmlRpcValue& pre_adjust, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(pre_adjust, tf_buffer, nh)
  {
    process_ = SET_GOAL;
    last_process_ = process_;
    process_num_ = 4;
    x_.init(pre_adjust["x"], "x", nh);
    y_.init(pre_adjust["y"], "y", nh);
    yaw_.init(pre_adjust["yaw"], "yaw", nh);
    ROS_INFO_STREAM("~~~~~~~~~~~~~PRE_ADJUST~~~~~~~~~~~~~~~~");
  }
  void init() override
  {
    ProgressBase::init();
    process_ = SET_GOAL;
  }
  void stateMachine() override
  {
    switch (process_)
    {
      case SET_GOAL:
      {
        set_goal();
        process_ = CHASSIS_Y;
      }
      break;
      case CHASSIS_Y:
      {
        computerChassisVel();
        chassis_vel_cmd_.linear.x = 0.;
        chassis_vel_cmd_.angular.z = 0.;
        if (y_.isFinish())
          process_ = CHASSIS_X;
      }
      break;
      case CHASSIS_X:
      {
        computerChassisVel();
        chassis_vel_cmd_.linear.y = 0.;
        chassis_vel_cmd_.angular.z = 0.;
        if (x_.isFinish())
          process_ = CHASSIS_YAW;
      }
      break;
      case CHASSIS_YAW:
      {
        computerChassisVel();
        chassis_vel_cmd_.linear.x = 0.;
        chassis_vel_cmd_.linear.y = 0.;
        if (yaw_.isFinish())
        {
          process_ = FINISH;
        }
      }
      break;
      case FINISH:
      {
        is_finish_ = true;
        initComputerValue();
        ROS_INFO_STREAM("PRE ADJUST FINISH");
      }
      break;
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
  void printProcess() override
  {
    ROS_INFO_STREAM("PRE ADJUST");
  }
  void computerChassisVel()
  {
    geometry_msgs::TransformStamped current;
    current = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    x_.error = chassis_target_.pose.position.x - current.transform.translation.x;
    y_.error = chassis_target_.pose.position.y - current.transform.translation.y;
    double roll, pitch, yaw_current, yaw_goal;
    quatToRPY(current.transform.rotation, roll, pitch, yaw_current);
    quatToRPY(chassis_target_.pose.orientation, roll, pitch, yaw_goal);
    yaw_.error = angles::shortest_angular_distance(yaw_current, yaw_goal);

    ros::Duration dt = ros::Time::now() - last_time_;
    chassis_vel_cmd_.linear.x = x_.computerVel(dt);
    chassis_vel_cmd_.linear.y = y_.computerVel(dt);
    chassis_vel_cmd_.angular.z = yaw_.computerVel(dt);

    last_time_ = ros::Time::now();
  }
  void set_goal()
  {
    geometry_msgs::TransformStamped base2exchange;
    double roll, pitch, yaw;
    base2exchange = tf_buffer_.lookupTransform("base_link", "exchanger", ros::Time(0));
    quatToRPY(base2exchange.transform.rotation, roll, pitch, yaw);

    double goal_x = base2exchange.transform.translation.x - x_.offset_refer_exchanger;
    double goal_y = base2exchange.transform.translation.y - y_.offset_refer_exchanger - yaw * 0.5;
    double goal_yaw = yaw * yaw_.offset_refer_exchanger;
    chassis_original_target_.pose.position.x = goal_x;
    chassis_original_target_.pose.position.y = goal_y;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, goal_yaw);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    chassis_original_target_.pose.orientation = quat_msg;
    chassis_target_ = chassis_original_target_;
    tf2::doTransform(chassis_target_, chassis_target_, tf_buffer_.lookupTransform("map", "base_link", ros::Time(0)));
  }

  ros::Time last_time_;
  std::string chassis_command_source_frame_{ "base_link" };
  geometry_msgs::Twist chassis_vel_cmd_{};
  geometry_msgs::PoseStamped chassis_target_{}, chassis_original_target_{};
  ChassisSingleDirectionMove x_, y_, yaw_;
};

class AutoServoMove : public ProgressBase
{
public:
  enum ServoMoveProcess
  {
    YZ,
    YAW,
    ROLL,
    PITCH,
    REY,
    REZ,
    PUSH,
    FINISH
  };
  AutoServoMove(XmlRpc::XmlRpcValue& auto_servo_move, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(auto_servo_move, tf_buffer, nh), joint7_msg_(0.)
  {
    ROS_ASSERT(auto_servo_move["xyz_offset"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["servo_error_tolerance"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(auto_servo_move["link7_length"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    process_ = YZ;
    last_process_ = process_;
    process_num_ = 8;

    xyz_offset_.resize(3, 0.);
    servo_pid_value_.resize(6, 0.);
    servo_errors_.resize(6, 0.);
    servo_scales_.resize(6, 0.);
    servo_error_tolerance_.resize(6, 0.01);
    ros::NodeHandle nh_servo_pid = ros::NodeHandle(nh, "servo_pid");
    ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_servo_pid, "x");
    ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_servo_pid, "y");
    ros::NodeHandle nh_pid_re_y = ros::NodeHandle(nh_servo_pid, "re_y");
    ros::NodeHandle nh_pid_re_z = ros::NodeHandle(nh_servo_pid, "re_z");
    ros::NodeHandle nh_pid_z = ros::NodeHandle(nh_servo_pid, "z");
    ros::NodeHandle nh_pid_roll = ros::NodeHandle(nh_servo_pid, "roll");
    ros::NodeHandle nh_pid_pitch = ros::NodeHandle(nh_servo_pid, "pitch");
    ros::NodeHandle nh_pid_yaw = ros::NodeHandle(nh_servo_pid, "yaw");
    pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
    pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
    pid_re_y_.init(ros::NodeHandle(nh_pid_re_y, "pid"));
    pid_re_z_.init(ros::NodeHandle(nh_pid_re_z, "pid"));
    pid_z_.init(ros::NodeHandle(nh_pid_z, "pid"));
    pid_roll_.init(ros::NodeHandle(nh_pid_roll, "pid"));
    pid_pitch_.init(ros::NodeHandle(nh_pid_pitch, "pid"));
    pid_yaw_.init(ros::NodeHandle(nh_pid_yaw, "pid"));

    close_tolerance_ = auto_servo_move["close_tolerance"];
    for (int i = 0; i < (int)xyz_offset_.size(); ++i)
      xyz_offset_[i] = auto_servo_move["xyz_offset"][i];
    for (int i = 0; i < (int)servo_error_tolerance_.size(); ++i)
    {
      servo_error_tolerance_[i] = auto_servo_move["servo_error_tolerance"][i];
    }
    ROS_INFO_STREAM("~~~~~~~~~~~~~SERVO_MOVE~~~~~~~~~~~~~~~~");
  }
  ~AutoServoMove() = default;
  void init() override
  {
    ProgressBase::init();
    process_ = { YZ };
    initComputerValue();
    joint7_msg_ = 0.;
  }
  double getJoint7Msg()
  {
    return joint7_msg_;
  }
  std::vector<double> getServoScale()
  {
    return servo_scales_;
  }

  void printProcess() override
  {
    if (process_ == YZ)
      ROS_INFO_STREAM("YZ");
    else if (process_ == YAW)
      ROS_INFO_STREAM("YAW");
    else if (process_ == ROLL)
      ROS_INFO_STREAM("ROLL");
    else if (process_ == PITCH)
      ROS_INFO_STREAM("PITCH");
    else if (process_ == REY)
      ROS_INFO_STREAM("REY");
    else if (process_ == REZ)
      ROS_INFO_STREAM("REZ");
    else if (process_ == PUSH)
      ROS_INFO_STREAM("PUSH");
    else if (process_ == FINISH)
      ROS_INFO_STREAM("FINISH");
  }

private:
  void stateMachine() override
  {
    computeServoMoveScale();
    manageServoMoveProcess();
  }
  void initComputerValue()
  {
    for (int i = 0; i < (int)servo_scales_.size(); ++i)
    {
      servo_errors_[i] = 0;
      servo_scales_[i] = 0;
      servo_pid_value_[i] = 0;
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
    ros::Duration dt = ros::Time::now() - last_time_;
    last_time_ = ros::Time::now();

    servo_pid_value_[0] = pid_x_.computeCommand(servo_errors_[0], dt);
    if (process_ != REY)
      servo_pid_value_[1] = pid_y_.computeCommand(servo_errors_[1], dt);
    else
      servo_pid_value_[1] = pid_re_y_.computeCommand(servo_errors_[1], dt);
    if (process_ != REZ)
      servo_pid_value_[2] = pid_z_.computeCommand(servo_errors_[2], dt);
    else
      servo_pid_value_[2] = pid_re_z_.computeCommand(servo_errors_[2], dt);
    servo_pid_value_[3] = pid_roll_.computeCommand(servo_errors_[3], dt);
    servo_pid_value_[4] = pid_pitch_.computeCommand(servo_errors_[4], dt);
    servo_pid_value_[5] = pid_yaw_.computeCommand(servo_errors_[5], dt);
  }
  void computeServoMoveScale()
  {
    computeServoMoveError();
    switch (process_)
    {
      case YZ:
      {
        servo_scales_[1] = servo_pid_value_[1];
        servo_scales_[2] = servo_pid_value_[2];
      }
      break;
      case YAW:
      {
        servo_scales_[5] = servo_pid_value_[5];
      }
      break;
      case ROLL:
      {
        servo_scales_[3] = servo_pid_value_[3];
      }
      break;
      case PITCH:
      {
        joint7_msg_ = servo_errors_[4];
      }
      break;
      case REY:
      {
        servo_scales_[1] = servo_pid_value_[1];
      }
      break;
      case REZ:
      {
        servo_scales_[2] = servo_pid_value_[2];
      }
      break;
      case PUSH:
      {
        servo_scales_[0] = servo_pid_value_[0];
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
        if (process_ == YZ)
        {
          if (abs(servo_errors_[i]) <= close_tolerance_)
            arrived_joint_num++;
        }
        else
        {
          if (abs(servo_errors_[i]) <= servo_error_tolerance_[i])
            arrived_joint_num++;
        }
      }
    }
    if (arrived_joint_num == move_joint_num)
    {
      if (process_ != FINISH)
        process_++;
      else
        is_finish_ = true;
    }
  }
  void rectifyForLink7(double theta, double link7_length)
  {
    rectify_x_ = link7_length_ * pow(sin(theta), 2) / (tan(M_PI_2 - theta / 2));
    rectify_z_ = link7_length_ * sin(theta) * cos(theta) / (tan(M_PI_2 - theta / 2));
  }

  ros::Time last_time_;
  double link7_length_{}, joint7_msg_{}, rectify_x_, rectify_z_, close_tolerance_;
  control_toolbox::Pid pid_x_, pid_y_, pid_re_y_, pid_re_z_, pid_z_, pid_roll_, pid_pitch_, pid_yaw_;
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

class AutoExchange : public ProgressBase
{
public:
  enum ExchangeProcess
  {
    FIND,
    PRE_ADJUST,
    MOVE,
    FINISH
  };
  AutoExchange(XmlRpc::XmlRpcValue& auto_exchange, tf2_ros::Buffer& tf_buffer, ros::NodeHandle& nh)
    : ProgressBase(auto_exchange, tf_buffer, nh)
  {
    process_ = FIND;
    last_process_ = process_;
    process_num_ = 4;
    ros::NodeHandle nh_auto_find(nh, "auto_find");
    ros::NodeHandle nh_auto_pre_adjust(nh, "auto_pre_adjust");
    ros::NodeHandle nh_auto_servo_move(nh, "auto_servo_move");
    find_ = new Find(auto_exchange["auto_find"], tf_buffer, nh_auto_find);
    pre_adjust_ = new ProAdjust(auto_exchange["auto_pre_adjust"], tf_buffer, nh_auto_pre_adjust);
    auto_servo_move_ = new AutoServoMove(auto_exchange["auto_servo_move"], tf_buffer, nh_auto_servo_move);
    exchanger_tf_update_pub_ = nh_.advertise<std_msgs::Bool>("/is_update_exchanger", 1);
  }
  void init() override
  {
    ProgressBase::init();
    process_ = FIND;
    re_find_ = false;
    find_->init();
    pre_adjust_->init();
    auto_servo_move_->init();
    exchangerTfUpdate(true);
  }

public:
  Find* find_{};
  ProAdjust* pre_adjust_{};
  AutoServoMove* auto_servo_move_{};

private:
  void exchangerTfUpdate(bool is_exchanger_tf_update)
  {
    is_exchanger_tf_update_.data = is_exchanger_tf_update;
    exchanger_tf_update_pub_.publish(is_exchanger_tf_update_);
  }
  void stateMachine() override
  {
    switch (process_)
    {
      case FIND:
      {
        exchangerTfUpdate(true);
        find_->run();
        if (find_->getFinishFlag())
        {
          process_ = re_find_ ? MOVE : PRE_ADJUST;
          find_->init();
        }
      }
      break;
      case PRE_ADJUST:
      {
        exchangerTfUpdate(true);
        pre_adjust_->run();
        if (pre_adjust_->getFinishFlag())
        {
          process_ = FIND;
          re_find_ = true;
          pre_adjust_->init();
        }
      }
      break;
      case MOVE:
      {
        exchangerTfUpdate(false);
        auto_servo_move_->run();
        auto_servo_move_->printProcess();
        if (auto_servo_move_->getFinishFlag())
        {
          is_finish_ = true;
          auto_servo_move_->init();
        }
      }
      break;
    }
  }
  void printProcess() override
  {
    if (process_ == FIND)
      ROS_INFO_STREAM("FIND");
    else if (process_ == PRE_ADJUST)
      ROS_INFO_STREAM("PRE_ADJUST");
    else if (process_ == MOVE)
      ROS_INFO_STREAM("MOVE");
    else if (process_ == FINISH)
      ROS_INFO_STREAM("FINISH");
  }
  bool re_find_{ false };
  // tf update
  std_msgs::Bool is_exchanger_tf_update_{};
  ros::Publisher exchanger_tf_update_pub_;
};
}  // namespace auto_exchange

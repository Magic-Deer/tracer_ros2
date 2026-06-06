/*
 * tracer_messenger.hpp
 *
 * Created on: Jun 14, 2022 16:37
 * Description:
 *
 * Copyright (c) 2022 tx (tx)
 */

#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>
#include <chrono>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tracer_msgs/msg/tracer_status.hpp"
#include "tracer_msgs/msg/tracer_light_cmd.hpp"
#include "tracer_msgs/msg/tracer_rc_state.hpp"

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
template <typename TracerType>
class TracerMessenger {
 public:
  TracerMessenger(std::shared_ptr<TracerType> tracer, rclcpp::Node *node)
      : tracer_(tracer), node_(node) {}

  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }
  void SetPortName(std::string name) { port_name_ = name; }

  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<tracer_msgs::msg::TracerStatus>(
        "tracer_status", 10);
    rc_status_pub_ = node_->create_publisher<tracer_msgs::msg::TracerRCState>(
        "tracer_rc_status",10);
    diagnostics_pub_ =
        node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
        
    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 5,
        std::bind(&TracerMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));
    light_cmd_sub_ = node_->create_subscription<tracer_msgs::msg::TracerLightCmd>(
        "light_control", 5,
        std::bind(&TracerMessenger::LightCmdCallback, this,
                  std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    last_time_ = node_->get_clock()->now();
    last_diagnostics_time_ =
        last_time_ - rclcpp::Duration::from_seconds(kDiagnosticsPeriodSec);
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();

    auto state = tracer_->GetRobotState();
    const double feedback_age_sec = GetFeedbackAgeSec(state);
    const auto lifecycle_state = GetLifecycleState(state, feedback_age_sec);
    if (lifecycle_state != lifecycle_state_) {
      RCLCPP_INFO(node_->get_logger(), "tracer/base state: %s",
                  LifecycleStateName(lifecycle_state));
    }
    lifecycle_state_ = lifecycle_state;

    PublishDiagnostics(state, feedback_age_sec, lifecycle_state_);

    if (lifecycle_state_ == LifecycleState::kConnecting) {
      MaybeEnableCommandedMode();
    }

    if (lifecycle_state_ != LifecycleState::kReady) {
      last_time_ = current_time_;
      return;
    }

    double dt = (current_time_ - last_time_).seconds();
    auto motion_state = state.motion_state;
    // Tracer reports forward motion as negative linear velocity.
    motion_state.linear_velocity = -motion_state.linear_velocity;

    // publish tracer state message
    tracer_msgs::msg::TracerStatus status_msg;
    tracer_msgs::msg::TracerRCState rc_state_msg;
    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = motion_state.linear_velocity;
    status_msg.angular_velocity = motion_state.angular_velocity;

    //status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    rc_state_msg.stick_left_h =   state.rc_state.stick_left_h;
    rc_state_msg.stick_left_v =   state.rc_state.stick_left_v;
    rc_state_msg.stick_right_h =   state.rc_state.stick_right_h;
    rc_state_msg.stick_right_v =   state.rc_state.stick_right_v;

    rc_state_msg.swa = state.rc_state.swa;
    rc_state_msg.swb = state.rc_state.swb;
    rc_state_msg.swc = state.rc_state.swc;
    rc_state_msg.swd = state.rc_state.swd;

    rc_state_msg.var_a = state.rc_state.var_a;
    
    rc_status_pub_->publish(rc_state_msg);


    auto actuator = tracer_->GetActuatorState();

    for (int i = 0; i < 2; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
    }

    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(motion_state, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

  void PublishSimStateToROS() {
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      twist_time_ = last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    tracer_msgs::msg::TracerStatus status_msg;

    status_msg.header.stamp = current_time_;
    status_msg.control_mode = 0x01;
    status_msg.error_code = 0x00;
    status_msg.battery_voltage = 29.5;
    status_msg.light_control_enabled = false;
    GetMotionForSim(status_msg.linear_velocity, status_msg.angular_velocity);
    status_pub_->publish(status_msg);

    MotionStateMessage motion_msg;
    motion_msg.linear_velocity = status_msg.linear_velocity;
    motion_msg.angular_velocity = status_msg.angular_velocity;
    PublishOdometryToROS(motion_msg, dt);

    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<TracerType> tracer_;
  rclcpp::Node *node_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  std::string port_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::mutex twist_mutex_;
  rclcpp::Time twist_time_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tracer_msgs::msg::TracerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<tracer_msgs::msg::TracerRCState>::SharedPtr rc_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<tracer_msgs::msg::TracerLightCmd>::SharedPtr
      light_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  rclcpp::Time last_diagnostics_time_;

  enum class LifecycleState {
    kOffline,
    kConnecting,
    kReady,
    kError,
  };

  LifecycleState lifecycle_state_ = LifecycleState::kOffline;
  SdkTimePoint last_commanded_mode_time_;

  static constexpr double kFeedbackTimeoutSec = 0.5;
  static constexpr double kCommandedModeRetryPeriodSec = 1.0;
  static constexpr double kDiagnosticsPeriodSec = 1.0;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!simulated_robot_) {
      if (lifecycle_state_ == LifecycleState::kReady) {
        SetTracerMotionCommand(msg);
      }
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      twist_time_ = node_->get_clock()->now();
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }
  void SetTracerMotionCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    tracer_->SetMotionCommand(msg->linear.x, msg->angular.z);
  }

  double GetFeedbackAgeSec(const TracerCoreState &state) const {
    if (state.time_stamp == SdkTimePoint{}) {
      return -1.0;
    }
    return std::chrono::duration<double>(SdkClock::now() - state.time_stamp)
        .count();
  }

  LifecycleState GetLifecycleState(const TracerCoreState &state,
                                   double feedback_age_sec) const {
    if (feedback_age_sec < 0.0 || feedback_age_sec > kFeedbackTimeoutSec) {
      return LifecycleState::kOffline;
    }
    if (state.system_state.error_code != 0) {
      return LifecycleState::kError;
    }
    if (state.system_state.control_mode != CONTROL_MODE_CAN) {
      return LifecycleState::kConnecting;
    }
    return LifecycleState::kReady;
  }

  void MaybeEnableCommandedMode() {
    const auto now = SdkClock::now();
    if (last_commanded_mode_time_ != SdkTimePoint{} &&
        std::chrono::duration<double>(now - last_commanded_mode_time_).count() <
            kCommandedModeRetryPeriodSec) {
      return;
    }
    tracer_->EnableCommandedMode();
    last_commanded_mode_time_ = now;
  }

  const char *LifecycleStateName(LifecycleState state) const {
    switch (state) {
      case LifecycleState::kOffline:
        return "offline";
      case LifecycleState::kConnecting:
        return "connecting";
      case LifecycleState::kReady:
        return "ready";
      case LifecycleState::kError:
        return "error";
    }
    return "unknown";
  }

  uint8_t DiagnosticsLevel(LifecycleState state) const {
    switch (state) {
      case LifecycleState::kReady:
        return diagnostic_msgs::msg::DiagnosticStatus::OK;
      case LifecycleState::kError:
        return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      case LifecycleState::kOffline:
      case LifecycleState::kConnecting:
        return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  diagnostic_msgs::msg::KeyValue MakeKeyValue(const std::string &key,
                                              const std::string &value) const {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    return pair;
  }

  void PublishDiagnostics(const TracerCoreState &state,
                          double feedback_age_sec,
                          LifecycleState lifecycle_state) {
    if ((current_time_ - last_diagnostics_time_).seconds() <
        kDiagnosticsPeriodSec) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "tracer/base";
    status.hardware_id = "tracer:" + port_name_;
    status.level = DiagnosticsLevel(lifecycle_state);
    status.message = LifecycleStateName(lifecycle_state);
    status.values = {
        MakeKeyValue("state", LifecycleStateName(lifecycle_state)),
        MakeKeyValue("port", port_name_),
        MakeKeyValue("feedback_age_sec", std::to_string(feedback_age_sec)),
        MakeKeyValue("control_mode",
                     std::to_string(static_cast<int>(
                         state.system_state.control_mode))),
        MakeKeyValue("battery_voltage",
                     std::to_string(state.system_state.battery_voltage)),
        MakeKeyValue("error_code",
                     std::to_string(state.system_state.error_code)),
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = current_time_;
    array.status.push_back(status);
    diagnostics_pub_->publish(array);

    last_diagnostics_time_ = current_time_;
  }

  void GetMotionForSim(double& linear, double& angular) {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
    double dt = (current_time_ - twist_time_).seconds();
    if (dt > 0.03) {
      linear = std::copysign(std::max(std::abs(linear) - 0.5*dt, 0.0), linear);
      angular = std::copysign(std::max(std::abs(angular) - dt, 0.0), angular);
    }
  }

  void LightCmdCallback(const tracer_msgs::msg::TracerLightCmd::SharedPtr msg) {
    if (!simulated_robot_) {
      if (msg->cmd_ctrl_allowed) {
        LightCommandMessage cmd;

        switch (msg->front_mode)
        {
          case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_OFF:
          {
            cmd.front_light.mode = CONST_OFF;
            break;
          }
          case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_ON:
          {
            cmd.front_light.mode = CONST_ON;
            break;
          }
          case tracer_msgs::msg::TracerLightCmd::LIGHT_BREATH:
          {
            cmd.front_light.mode = BREATH;
            break;
          }
          case tracer_msgs::msg::TracerLightCmd::LIGHT_CUSTOM:
          {
            cmd.front_light.mode = CUSTOM;
            cmd.front_light.custom_value = msg->front_custom_value;
            break;
          }
        }
        tracer_->SetLightCommand(cmd.front_light.mode,cmd.front_light.custom_value);
      }
    } else {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
  }
  

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed = msg.linear_velocity;
    double angular_speed = msg.angular_velocity;

    // if (std::is_base_of<TracerMiniOmniRobot, TracerType>::value) {
    //   lateral_speed = msg.lateral_velocity;
    // } else {
    //   lateral_speed = 0;
    // }

    double d_x = linear_speed * std::cos(theta_) * dt;
    double d_y = linear_speed * std::sin(theta_) * dt;
    double d_theta = angular_speed * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed;

    odom_pub_->publish(odom_msg);
  }
};
}  // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */

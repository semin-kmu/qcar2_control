#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "qcar2_msgs/msg/lateral_guidance.hpp"
#include "qcar2_msgs/msg/ph_quintic_path.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_control/ph_runtime.hpp"

namespace ph = qcar2_control::ph;

static constexpr double kEpsilon = 1e-6;
static constexpr double kMaxDt = 1.0;
static constexpr double kMaxSteerRad = 0.6;

class PidLateralControllerPHNode : public rclcpp::Node
{
public:
  PidLateralControllerPHNode()
  : Node("pid_lateral_controller_ph_node")
  {
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    kp_ = this->declare_parameter<double>("kp", 0.8);
    ki_ = this->declare_parameter<double>("ki", 0.0);
    kd_ = this->declare_parameter<double>("kd", 0.12);
    kff_ = this->declare_parameter<double>("kff", 0.85);
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.257);
    d_filter_alpha_ = this->declare_parameter<double>("d_filter_alpha", 0.25);
    i_min_ = this->declare_parameter<double>("i_min", -0.5);
    i_max_ = this->declare_parameter<double>("i_max", 0.5);
    steer_rate_limit_radps_ = this->declare_parameter<double>("steer_rate_limit_radps", 10.0);
    guidance_timeout_sec_ = this->declare_parameter<double>("guidance_timeout_sec", 0.5);
    throttle_test_value_mps_ = this->declare_parameter<double>("throttle_test_value_mps", 0.0);
    stop_on_penultimate_ = this->declare_parameter<bool>("stop_on_penultimate", true);
    stop_distance_m_ = this->declare_parameter<double>("stop_distance_m", 0.1);
    stop_sample_count_ = this->declare_parameter<int>("stop_sample_count", 200);

    gain_schedule_enabled_ = this->declare_parameter<bool>("gain_schedule_enabled", true);
    gain_ref_speed_ = this->declare_parameter<double>("gain_ref_speed", 0.3);
    gain_min_speed_ = this->declare_parameter<double>("gain_min_speed", 0.2);

    speed_control_enabled_ = this->declare_parameter<bool>("speed_control_enabled", true);
    speed_max_ = this->declare_parameter<double>("speed_max", 0.6);
    speed_min_ = this->declare_parameter<double>("speed_min", 0.2);
    curvature_low_ = this->declare_parameter<double>("curvature_low", 0.3);
    curvature_high_ = this->declare_parameter<double>("curvature_high", 0.7);
    max_acceleration_ = this->declare_parameter<double>("max_acceleration", 0.2);
    max_deceleration_ = this->declare_parameter<double>("max_deceleration", 0.4);

    speed_smooth_alpha_ = this->declare_parameter<double>("speed_smooth_alpha", 0.5);
    steer_smooth_alpha_ = this->declare_parameter<double>("steer_smooth_alpha", 0.9);

    sub_guidance_ = this->create_subscription<qcar2_msgs::msg::LateralGuidance>(
      "/vfg/lateral_guidance", rclcpp::QoS(10),
      std::bind(&PidLateralControllerPHNode::guidanceCallback, this, std::placeholders::_1));

    sub_path_ = this->create_subscription<qcar2_msgs::msg::PhQuinticPath>(
      "/planning/local_path_ph", rclcpp::QoS(1),
      std::bind(&PidLateralControllerPHNode::pathCallback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, control_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PidLateralControllerPHNode::update, this));

    cmd_msg_.motor_names = {"steering_angle", "motor_throttle"};
    cmd_msg_.values.resize(2);

    RCLCPP_INFO(this->get_logger(), "PID PH lateral controller started");
  }

private:
  void guidanceCallback(const qcar2_msgs::msg::LateralGuidance::SharedPtr msg)
  {
    latest_guidance_ = *msg;
    have_guidance_ = true;
    last_guidance_time_ = this->now();

    if (!have_curve_id_ || msg->curve_id != last_curve_id_) {
      last_curve_id_ = msg->curve_id;
      have_curve_id_ = true;
      reset_pid_ = true;
      stopped_ = false;
    }
  }

  void pathCallback(const qcar2_msgs::msg::PhQuinticPath::SharedPtr msg)
  {
    if (!stop_on_penultimate_) {
      have_stop_u_ = false;
      return;
    }

    std::string reason;
    if (!stop_runtime_.updateFromMsg(*msg, reason)) {
      have_stop_u_ = false;
      return;
    }

    stop_curve_id_ = static_cast<uint32_t>(msg->traj_id);

    const int count = std::max(20, stop_sample_count_);
    stop_u_ = static_cast<double>(count - 2) / static_cast<double>(count - 1);
    stop_curve_point_ = stop_runtime_.pointAtRatio(stop_u_);
    have_stop_u_ = true;
  }

  void resetPidState()
  {
    integrator_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
    last_steer_cmd_ = 0.0;
    reset_pid_ = false;
    have_prev_guidance_stamp_ = false;
  }

  bool guidanceTimedOut(const rclcpp::Time & now) const
  {
    if (!have_guidance_) {
      return true;
    }
    const double dt = (now - last_guidance_time_).seconds();
    return dt > guidance_timeout_sec_;
  }

  double computeSpeedFromCurvature(double curvature) const
  {
    const double kappa = std::fabs(curvature);

    if (kappa <= curvature_low_) {
      return speed_max_;
    }
    if (kappa >= curvature_high_) {
      return speed_min_;
    }

    const double t = (kappa - curvature_low_) / (curvature_high_ - curvature_low_);
    const double smooth_t = t * t * (3.0 - 2.0 * t);
    return speed_max_ - (speed_max_ - speed_min_) * smooth_t;
  }

  void update()
  {
    const rclcpp::Time now = this->now();
    double dt_control = 0.0;
    if (have_last_control_time_) {
      dt_control = (now - last_control_time_).seconds();
      if (dt_control < kEpsilon || dt_control > kMaxDt) {
        dt_control = 0.0;
      }
    }
    last_control_time_ = now;
    have_last_control_time_ = true;

    if (guidanceTimedOut(now) ||
      !have_guidance_ ||
      latest_guidance_.status != qcar2_msgs::msg::LateralGuidance::STATUS_OK)
    {
      resetPidState();
      publishSmoothedCmd(dt_control, 0.0, 0.0);
      return;
    }

    if (stopped_) {
      publishStopCmd();
      return;
    }

    if (stop_on_penultimate_ && have_stop_u_ &&
      latest_guidance_.curve_id == stop_curve_id_)
    {
      const ph::Vec2 q{
        latest_guidance_.closest_point.x,
        latest_guidance_.closest_point.y
      };
      const double dist = ph::norm(q - stop_curve_point_);
      if (latest_guidance_.u_star >= stop_u_ && dist <= stop_distance_m_) {
        stopped_ = true;
        resetPidState();
        publishStopCmd();
        RCLCPP_INFO(this->get_logger(), "Vehicle stopped at target location (PH)");
        return;
      }
    }

    const double e = latest_guidance_.heading_error_rad;
    double dt_pid = dt_control;

    if ((latest_guidance_.header.stamp.sec != 0) ||
      (latest_guidance_.header.stamp.nanosec != 0))
    {
      rclcpp::Time stamp(latest_guidance_.header.stamp);
      if (have_prev_guidance_stamp_ && stamp > prev_guidance_stamp_) {
        const double dt = (stamp - prev_guidance_stamp_).seconds();
        if (dt > kEpsilon && dt < kMaxDt) {
          dt_pid = dt;
        }
      }
      prev_guidance_stamp_ = stamp;
      have_prev_guidance_stamp_ = true;
    }

    if (reset_pid_) {
      integrator_ = 0.0;
      prev_error_ = e;
      filtered_derivative_ = 0.0;
      reset_pid_ = false;
    }

    double target_speed = throttle_test_value_mps_;
    if (speed_control_enabled_ && std::isfinite(latest_guidance_.lookahead_max_curvature_1pm)) {
      target_speed = computeSpeedFromCurvature(latest_guidance_.lookahead_max_curvature_1pm);
    }

    double gain_scale = 1.0;
    if (gain_schedule_enabled_) {
      const double current_speed = std::fabs(target_speed);
      const double effective_speed = std::max(gain_min_speed_, current_speed);
      gain_scale = gain_ref_speed_ / effective_speed;
      gain_scale = ph::clamp(gain_scale, 0.1, 1.1);
    }

    const double p_term = gain_scale * kp_ * e;

    double d_term = 0.0;
    if (dt_pid > kEpsilon) {
      integrator_ += e * dt_pid;
      integrator_ = ph::clamp(integrator_, i_min_, i_max_);

      const double raw_derivative = (e - prev_error_) / dt_pid;
      filtered_derivative_ = d_filter_alpha_ * raw_derivative +
        (1.0 - d_filter_alpha_) * filtered_derivative_;
      d_term = gain_scale * kd_ * filtered_derivative_;
    }
    const double i_term = gain_scale * ki_ * integrator_;

    double ff_term = 0.0;
    if (std::isfinite(latest_guidance_.ff_lookahead_curvature_1pm)) {
      ff_term = kff_ * wheelbase_ * latest_guidance_.ff_lookahead_curvature_1pm;
    }

    const double steer_raw = p_term + i_term + d_term + ff_term;

    double steer_cmd = ph::clamp(steer_raw, -kMaxSteerRad, kMaxSteerRad);
    if (dt_control > kEpsilon && steer_rate_limit_radps_ > kEpsilon) {
      const double max_delta = steer_rate_limit_radps_ * dt_control;
      const double delta = steer_cmd - last_steer_cmd_;
      if (std::fabs(delta) > max_delta) {
        steer_cmd = last_steer_cmd_ + std::copysign(max_delta, delta);
      }
    }
    last_steer_cmd_ = steer_cmd;
    prev_error_ = e;

    publishSmoothedCmd(dt_control, target_speed, steer_cmd);
  }

  void publishSmoothedCmd(double dt, double target_speed, double target_steer)
  {
    double speed_cmd = last_speed_cmd_;
    if (dt > kEpsilon) {
      const double speed_delta = target_speed - last_speed_cmd_;
      if (speed_delta > 0) {
        const double max_delta = max_acceleration_ * dt;
        speed_cmd = (speed_delta > max_delta) ? (last_speed_cmd_ + max_delta) : target_speed;
      } else {
        const double max_delta = max_deceleration_ * dt;
        speed_cmd = (-speed_delta > max_delta) ? (last_speed_cmd_ - max_delta) : target_speed;
      }
      last_speed_cmd_ = speed_cmd;
    }

    smoothed_speed_ = speed_smooth_alpha_ * speed_cmd + (1.0 - speed_smooth_alpha_) * smoothed_speed_;
    smoothed_steer_ = steer_smooth_alpha_ * target_steer + (1.0 - steer_smooth_alpha_) * smoothed_steer_;

    cmd_msg_.values[0] = ph::clamp(smoothed_steer_, -kMaxSteerRad, kMaxSteerRad);
    cmd_msg_.values[1] = ph::clamp(smoothed_speed_, -1.0, 1.0);
    pub_cmd_->publish(cmd_msg_);
  }

  void publishStopCmd()
  {
    smoothed_speed_ = 0.0;
    smoothed_steer_ = 0.0;
    last_speed_cmd_ = 0.0;
    last_steer_cmd_ = 0.0;

    cmd_msg_.values[0] = 0.0;
    cmd_msg_.values[1] = 0.0;
    pub_cmd_->publish(cmd_msg_);
  }

  double control_rate_hz_{200.0};
  double kp_{0.8};
  double ki_{0.0};
  double kd_{0.12};
  double kff_{0.85};
  double wheelbase_{0.257};
  double d_filter_alpha_{0.25};
  double i_min_{-0.5};
  double i_max_{0.5};
  double steer_rate_limit_radps_{10.0};
  double guidance_timeout_sec_{0.5};
  double throttle_test_value_mps_{0.0};
  bool stop_on_penultimate_{true};
  double stop_distance_m_{0.1};
  int stop_sample_count_{200};

  bool gain_schedule_enabled_{true};
  double gain_ref_speed_{0.3};
  double gain_min_speed_{0.2};

  bool speed_control_enabled_{true};
  double speed_max_{0.6};
  double speed_min_{0.2};
  double curvature_low_{0.3};
  double curvature_high_{0.7};
  double max_acceleration_{0.2};
  double max_deceleration_{0.4};

  double speed_smooth_alpha_{0.5};
  double steer_smooth_alpha_{0.9};

  double last_speed_cmd_{0.0};
  double smoothed_speed_{0.0};
  double smoothed_steer_{0.0};
  bool stopped_{false};

  ph::PhPathRuntime stop_runtime_;
  uint32_t stop_curve_id_{0};
  bool have_stop_u_{false};
  double stop_u_{1.0};
  ph::Vec2 stop_curve_point_{0.0, 0.0};

  rclcpp::Subscription<qcar2_msgs::msg::LateralGuidance>::SharedPtr sub_guidance_;
  rclcpp::Subscription<qcar2_msgs::msg::PhQuinticPath>::SharedPtr sub_path_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  qcar2_msgs::msg::LateralGuidance latest_guidance_;
  bool have_guidance_{false};
  rclcpp::Time last_guidance_time_{0, 0, RCL_ROS_TIME};

  bool have_curve_id_{false};
  uint32_t last_curve_id_{0};
  bool reset_pid_{false};

  double integrator_{0.0};
  double prev_error_{0.0};
  double filtered_derivative_{0.0};
  double last_steer_cmd_{0.0};

  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool have_last_control_time_{false};

  rclcpp::Time prev_guidance_stamp_{0, 0, RCL_ROS_TIME};
  bool have_prev_guidance_stamp_{false};

  qcar2_interfaces::msg::MotorCommands cmd_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidLateralControllerPHNode>());
  rclcpp::shutdown();
  return 0;
}

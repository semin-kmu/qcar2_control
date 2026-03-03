#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2/time.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "qcar2_msgs/msg/bezier_curve.hpp"
#include "qcar2_msgs/msg/lateral_guidance.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_control/bezier.hpp"

// File-local numerical constants for VFG geometry/numeric checks.
static constexpr double kEpsilon   = 1e-9;
static constexpr double kEpsilonSq = 1e-12;

class VFGGuidanceNode : public rclcpp::Node
{
public:
  VFGGuidanceNode()
  : Node("vfg_guidance_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 100.0);
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.2);
    subdivide_max_depth_ = this->declare_parameter<int>("subdivide_max_depth", 6);
    subdivide_flatness_m_ = this->declare_parameter<double>("subdivide_flatness_m", 0.02);
    u_search_window_ = this->declare_parameter<double>("u_search_window", 0.2);

    a0_ = this->declare_parameter<double>("a0", 0.2);    // 0.637 = 2/pi

    // Lookahead curvature parameters.
    lookahead_preview_time_ = this->declare_parameter<double>("lookahead_preview_time", 1.3);
    lookahead_min_distance_ = this->declare_parameter<double>("lookahead_min_distance", 0.1);
    lookahead_max_distance_ = this->declare_parameter<double>("lookahead_max_distance", 1.2);
    lookahead_sample_count_ = this->declare_parameter<int>("lookahead_sample_count", 20);

    // Feed-forward lookahead parameter (fixed distance).
    ff_lookahead_distance_ = this->declare_parameter<double>("ff_lookahead_distance", 0.13);  // base_link + wheelbase + small offset

    publish_markers_ = this->declare_parameter<bool>("publish_markers", false);
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    sub_path_ = this->create_subscription<qcar2_msgs::msg::BezierCurve>(
      "/planning/local_path", rclcpp::QoS(1),
      std::bind(&VFGGuidanceNode::pathCallback, this, std::placeholders::_1));

    sub_motor_cmd_ = this->create_subscription<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", rclcpp::QoS(10),
      std::bind(&VFGGuidanceNode::motorCmdCallback, this, std::placeholders::_1));

    pub_guidance_ = this->create_publisher<qcar2_msgs::msg::LateralGuidance>(
      "/vfg/lateral_guidance", rclcpp::QoS(10));

    if (publish_markers_) {
      pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vfg/debug_markers", rclcpp::QoS(10));
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, update_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VFGGuidanceNode::update, this));

    RCLCPP_INFO(this->get_logger(), "VFG guidance node started");
  }

private:
  void pathCallback(const qcar2_msgs::msg::BezierCurve::SharedPtr msg)
  {
    has_curve_ = true;
    invalid_reason_.clear();
    frame_id_ = msg->header.frame_id;
    curve_id_ = msg->curve_id;
    degree_ = static_cast<int>(msg->degree);

    if (frame_id_.empty()) {
      has_valid_curve_ = false;
      invalid_reason_ = "Empty frame_id";
      return;
    }

    if (degree_ < 1 || msg->control_points.size() != static_cast<size_t>(degree_ + 1)) {
      has_valid_curve_ = false;
      invalid_reason_ = "Invalid degree/control_points";
      return;
    }

    control_points_.clear();
    control_points_.reserve(msg->control_points.size());
    for (const auto & p : msg->control_points) {
      control_points_.push_back({p.x, p.y});
    }

    // Cache control-polygon length once; reused every update() without re-traversal.
    cached_approx_length_ = 0.0;
    for (size_t i = 1; i < control_points_.size(); ++i) {
      cached_approx_length_ += norm(control_points_[i] - control_points_[i - 1]);
    }

    binom_ = computeBinom(degree_);
    binom_d_ = computeBinom(std::max(0, degree_ - 1));
    binom_dd_ = computeBinom(std::max(0, degree_ - 2));

    // Initialize subdivision buffers with per-depth preallocation.
    const size_t n = control_points_.size();
    const size_t max_depth = static_cast<size_t>(subdivide_max_depth_) + 1;
    split_left_buffer_.resize(max_depth);
    split_right_buffer_.resize(max_depth);
    split_tmp_buffer_.resize(max_depth);
    for (size_t d = 0; d < max_depth; ++d) {
      split_left_buffer_[d].reserve(n);
      split_right_buffer_[d].reserve(n);
      split_tmp_buffer_[d].reserve(n);
    }

    have_last_u_ = false;
    has_valid_curve_ = true;
  }

  void motorCmdCallback(const qcar2_interfaces::msg::MotorCommands::SharedPtr msg)
  {
    // Extract motor_throttle from MotorCommands.
    if (msg->motor_names.size() >= 2 && msg->values.size() >= 2) {
      for (size_t i = 0; i < msg->motor_names.size(); ++i) {
        if (msg->motor_names[i] == "motor_throttle") {
          current_speed_cmd_ = std::fabs(msg->values[i]);
          have_speed_cmd_ = true;
          return;
        }
      }
    }
  }

  // Thin wrappers: delegate Bezier evaluation to free functions in bezier.hpp.
  Vec2 evalBezier(double u) const
  {
    return ::evalBezier(u, control_points_, binom_, degree_);
  }

  Vec2 evalBezierDerivative(double u) const
  {
    return ::evalBezierDerivative(u, control_points_, binom_d_, degree_);
  }

  Vec2 evalBezierSecondDerivative(double u) const
  {
    return ::evalBezierSecondDerivative(u, control_points_, binom_dd_, degree_);
  }

  struct ClosestPointResult
  {
    double dist{std::numeric_limits<double>::infinity()};
    double u{0.0};
    Vec2 point{0.0, 0.0};
  };

  void recursiveClosestPoint(
    const std::vector<Vec2> & points,
    const Vec2 & p,
    double u0,
    double u1,
    int depth,
    double u_min,
    double u_max,
    ClosestPointResult & best) const
  {
    if (points.size() < 2) {
      return;
    }
    if (u1 < u_min || u0 > u_max) {
      return;
    }

    const double flatness = controlPolygonFlatness(points);
    if (depth >= subdivide_max_depth_ || flatness <= subdivide_flatness_m_) {
      double t = 0.0;
      closestPointOnSegment(p, points.front(), points.back(), t);
      const double u = u0 + t * (u1 - u0);
      const Vec2 q = evalBezier(u);
      const double dist = norm(q - p);
      if (dist < best.dist) {
        best.dist = dist;
        best.u = u;
        best.point = q;
      }
      return;
    }

    // Use per-depth preallocated buffers to avoid conflicts during recursion.
    const size_t d = static_cast<size_t>(depth);
    std::vector<Vec2> & left = split_left_buffer_[d];
    std::vector<Vec2> & right = split_right_buffer_[d];
    std::vector<Vec2> & tmp = split_tmp_buffer_[d];
    deCasteljauSplit(points, left, right, tmp);
    const double umid = 0.5 * (u0 + u1);

    recursiveClosestPoint(left, p, u0, umid, depth + 1, u_min, u_max, best);
    recursiveClosestPoint(right, p, umid, u1, depth + 1, u_min, u_max, best);
  }

  bool findClosestPoint(const Vec2 & p, double & u_star, Vec2 & q_star) const
  {
    if (control_points_.size() < 2) {
      return false;
    }

    ClosestPointResult best;
    double u_min = 0.0;
    double u_max = 1.0;
    if (have_last_u_ && u_search_window_ > 0.0) {
      u_min = clamp(last_u_star_ - u_search_window_, 0.0, 1.0);
      u_max = clamp(last_u_star_ + u_search_window_, 0.0, 1.0);
      if (u_max < u_min + kEpsilon) {
        u_min = 0.0;
        u_max = 1.0;
      }
    }
    recursiveClosestPoint(control_points_, p, 0.0, 1.0, 0, u_min, u_max, best);
    if (!std::isfinite(best.dist)) {
      return false;
    }

    u_star = clamp(best.u, 0.0, 1.0);
    q_star = best.point;
    return true;
  }

  double computeCurvature(const Vec2 & d1, const Vec2 & d2) const
  {
    const double denom = std::pow(d1.x * d1.x + d1.y * d1.y, 1.5);
    if (denom < kEpsilonSq) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double numer = d1.x * d2.y - d1.y * d2.x;
    return numer / denom;
  }

  double computeCurvatureAtU(double u) const
  {
    const Vec2 d1 = evalBezierDerivative(u);
    const Vec2 d2 = evalBezierSecondDerivative(u);
    return computeCurvature(d1, d2);
  }

  // Compute maximum curvature over the forward lookahead span from u_start.
  // Returns: (approx_lookahead_distance, max_curvature).
  // approx_lookahead_distance is estimated from the cached control-polygon length
  // to avoid O(sample_count) extra evalBezier calls for arc-length tracking.
  std::pair<double, double> computeLookaheadMaxCurvature(double u_start, double lookahead_dist) const
  {
    if (control_points_.size() < 2 || lookahead_dist < kEpsilon) {
      return {0.0, std::numeric_limits<double>::quiet_NaN()};
    }

    // Use cached control-polygon length (computed once in pathCallback).
    const double approx_length = cached_approx_length_;
    if (approx_length < kEpsilon) {
      return {0.0, std::numeric_limits<double>::quiet_NaN()};
    }

    // Approximate lookahead distance as a u-domain range.
    const double du_per_meter = 1.0 / approx_length;
    const double u_end = std::min(1.0, u_start + lookahead_dist * du_per_meter);

    if (u_end <= u_start + kEpsilon) {
      return {0.0, std::numeric_limits<double>::quiet_NaN()};
    }

    // Sample along the interval and keep the maximum absolute curvature.
    // Arc-length is approximated from u-domain span to avoid extra evalBezier calls.
    const int sample_count = std::max(2, lookahead_sample_count_);
    double max_abs_curvature = 0.0;
    double max_curvature = 0.0;

    for (int i = 1; i <= sample_count; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(sample_count);
      const double u = u_start + t * (u_end - u_start);

      const double kappa = computeCurvatureAtU(u);
      if (std::isfinite(kappa) && std::fabs(kappa) > max_abs_curvature) {
        max_abs_curvature = std::fabs(kappa);
        max_curvature = kappa;
      }
    }

    const double approx_lookahead = (u_end - u_start) * approx_length;
    return {approx_lookahead, max_curvature};
  }

  void publishStatus(uint8_t status, const std::string & msg)
  {
    qcar2_msgs::msg::LateralGuidance out;
    out.header.stamp = this->now();
    out.header.frame_id = frame_id_;
    out.curve_id = curve_id_;
    out.status = status;
    out.status_msg = msg;
    out.curvature_1pm = std::numeric_limits<double>::quiet_NaN();
    pub_guidance_->publish(out);

    if (publish_markers_) {
      visualization_msgs::msg::MarkerArray ma;
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(clear);
      pub_markers_->publish(ma);
    }
  }

  void update()
  {
    if (!has_curve_) {
      publishStatus(qcar2_msgs::msg::LateralGuidance::STATUS_NO_PATH, "No path received");
      return;
    }

    if (!has_valid_curve_) {
      publishStatus(qcar2_msgs::msg::LateralGuidance::STATUS_INVALID_PATH,
        invalid_reason_.empty() ? "Invalid path" : invalid_reason_);
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        frame_id_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      publishStatus(qcar2_msgs::msg::LateralGuidance::STATUS_TF_FAIL, ex.what());
      return;
    }

    const Vec2 p_vehicle{tf.transform.translation.x, tf.transform.translation.y};
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    const double psi_vehicle = tf2::getYaw(q);

    double u_star = 0.0;
    Vec2 q_star;
    if (!findClosestPoint(p_vehicle, u_star, q_star)) {
      publishStatus(qcar2_msgs::msg::LateralGuidance::STATUS_INVALID_PATH, "Empty path samples");
      return;
    }
    last_u_star_ = u_star;
    have_last_u_ = true;

    const Vec2 v2 = evalBezierDerivative(u_star);
    const double v2_norm = norm(v2);
    if (v2_norm < kEpsilon || !std::isfinite(v2_norm)) {
      publishStatus(qcar2_msgs::msg::LateralGuidance::STATUS_NUMERIC_FAIL, "Zero tangent");
      return;
    }

    const Vec2 v2_hat = normalize(v2);
    const Vec2 n_hat{-v2_hat.y, v2_hat.x};

    const Vec2 v1 = q_star - p_vehicle;
    const double v1_norm = norm(v1);
    Vec2 v1_hat{0.0, 0.0};
    if (v1_norm > kEpsilon && std::isfinite(v1_norm)) {
      v1_hat = {v1.x / v1_norm, v1.y / v1_norm};
    }
    const double cross_track_error = dot(v1, n_hat);

    // Arctangent weighting for smooth transition and reduced high-speed oscillation.
    // w1 = (2/π) * atan(|e| / a0)
    // e=0: w1=0 (pure tangent tracking), e=a0: w1=0.5, e->inf: w1->1.
    const double k_atan = 1.0 / std::max(kEpsilon, a0_);
    const double w1 = (2.0 / M_PI) * std::atan(k_atan * std::fabs(cross_track_error));
    const double w2 = std::sqrt(std::max(0.0, 1.0 - w1 * w1));

    const Vec2 v_vfg = w1 * v1_hat + w2 * v2_hat;
    const double v_vfg_norm = norm(v_vfg);
    if (v_vfg_norm < kEpsilon || !std::isfinite(v_vfg_norm)) {
      publishStatus(qcar2_msgs::msg::LateralGuidance::STATUS_NUMERIC_FAIL, "Invalid VFG vector");
      return;
    }

    const double psi_vfg = std::atan2(v_vfg.y, v_vfg.x);
    const double psi_path_tangent = std::atan2(v2_hat.y, v2_hat.x);
    const double heading_error = wrapToPi(psi_vfg - psi_vehicle);

    const Vec2 d2 = evalBezierSecondDerivative(u_star);
    const double curvature = computeCurvature(v2, d2);

    // Compute maximum curvature within the speed-based lookahead window.
    double lookahead_distance = lookahead_min_distance_;
    double lookahead_max_curvature = std::numeric_limits<double>::quiet_NaN();
    if (have_speed_cmd_) {
      lookahead_distance = current_speed_cmd_ * lookahead_preview_time_;
      lookahead_distance = clamp(lookahead_distance, lookahead_min_distance_, lookahead_max_distance_);
    }
    const auto [actual_lookahead, max_curv] = computeLookaheadMaxCurvature(u_star, lookahead_distance);
    lookahead_distance = actual_lookahead;
    lookahead_max_curvature = max_curv;

    // Compute fixed-distance lookahead curvature for feed-forward steering.
    // Uses cached control-polygon length (computed once in pathCallback).
    double ff_lookahead_curvature = std::numeric_limits<double>::quiet_NaN();
    if (control_points_.size() >= 2 && ff_lookahead_distance_ > kEpsilon &&
        cached_approx_length_ > kEpsilon) {
      const double du_per_meter = 1.0 / cached_approx_length_;
      const double u_ff = std::min(1.0, u_star + ff_lookahead_distance_ * du_per_meter);
      ff_lookahead_curvature = computeCurvatureAtU(u_ff);
    }

    qcar2_msgs::msg::LateralGuidance out;
    out.header.stamp = this->now();
    out.header.frame_id = frame_id_;
    out.curve_id = curve_id_;
    out.heading_error_rad = heading_error;
    out.u_star = u_star;
    out.closest_point.x = q_star.x;
    out.closest_point.y = q_star.y;
    out.closest_point.z = 0.0;
    out.psi_vehicle_rad = psi_vehicle;
    out.psi_path_tangent_rad = psi_path_tangent;
    out.psi_vfg_rad = psi_vfg;
    out.cross_track_error_m = cross_track_error;
    out.curvature_1pm = curvature;
    out.lookahead_distance_m = lookahead_distance;
    out.lookahead_max_curvature_1pm = lookahead_max_curvature;
    out.ff_lookahead_curvature_1pm = ff_lookahead_curvature;
    out.w1_to_path = w1;
    out.w2_tangent = w2;
    out.v1_to_path_norm = v1_norm;
    out.v_vfg_norm = v_vfg_norm;
    out.status = qcar2_msgs::msg::LateralGuidance::STATUS_OK;
    out.status_msg = "OK";
    pub_guidance_->publish(out);

    if (publish_markers_) {
      publishMarkers(p_vehicle, q_star, v2_hat, n_hat, v1, v2, v_vfg);
    }
  }

  void publishMarkers(
    const Vec2 & p, const Vec2 & q, const Vec2 & v2_hat, const Vec2 & n_hat,
    const Vec2 & v1, const Vec2 & v2, const Vec2 & v_vfg)
  {
    visualization_msgs::msg::MarkerArray ma;
    const rclcpp::Time stamp = this->now();

    auto makeArrow = [&](int id, const Vec2 & dir, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = stamp;
      m.ns = "vfg";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.02;  // shaft diameter
      m.scale.y = 0.05;  // head diameter
      m.scale.z = 0.08;  // head length
      m.color.a = 1.0f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;

      geometry_msgs::msg::Point p0;
      p0.x = p.x;
      p0.y = p.y;
      p0.z = 0.05;

      const Vec2 u = normalize(dir);
      geometry_msgs::msg::Point p1;
      p1.x = p.x + 0.5 * u.x;
      p1.y = p.y + 0.5 * u.y;
      p1.z = 0.05;

      m.points = {p0, p1};
      return m;
    };

    auto makeSphere = [&](int id, const Vec2 & pos, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = stamp;
      m.ns = "vfg";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.08;
      m.scale.y = 0.08;
      m.scale.z = 0.08;
      m.color.a = 1.0f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.pose.position.x = pos.x;
      m.pose.position.y = pos.y;
      m.pose.position.z = 0.05;
      return m;
    };

    ma.markers.push_back(makeSphere(0, q, 1.0f, 0.5f, 0.1f));      // closest point
    ma.markers.push_back(makeArrow(1, v2_hat, 0.2f, 1.0f, 0.2f));  // tangent
    ma.markers.push_back(makeArrow(2, n_hat, 1.0f, 0.2f, 0.2f));   // normal
    ma.markers.push_back(makeArrow(3, v1, 0.2f, 0.6f, 1.0f));      // to-path
    ma.markers.push_back(makeArrow(4, v2, 0.7f, 0.7f, 0.7f));      // tangent vector
    ma.markers.push_back(makeArrow(5, v_vfg, 0.1f, 0.3f, 1.0f));   // vfg

    pub_markers_->publish(ma);
  }

  double update_rate_hz_{20.0};
  double tf_timeout_sec_{0.2};
  int subdivide_max_depth_{6};
  double subdivide_flatness_m_{0.02};
  double u_search_window_{0.2};
  double a0_{0.18};
  double lookahead_preview_time_{1.2};
  double lookahead_min_distance_{0.3};
  double lookahead_max_distance_{1.0};
  int lookahead_sample_count_{15};
  double ff_lookahead_distance_{0.15};
  bool publish_markers_{true};
  std::string base_frame_{"base_link"};
  std::string frame_id_;

  bool has_valid_curve_{false};
  bool has_curve_{false};
  std::string invalid_reason_;
  uint32_t curve_id_{0};
  int degree_{0};
  bool have_last_u_{false};
  double last_u_star_{0.0};
  std::vector<Vec2> control_points_;
  std::vector<double> binom_;
  std::vector<double> binom_d_;
  std::vector<double> binom_dd_;

  // Per-depth buffer pools to avoid reallocations during recursive subdivision.
  mutable std::vector<std::vector<Vec2>> split_left_buffer_;
  mutable std::vector<std::vector<Vec2>> split_right_buffer_;
  mutable std::vector<std::vector<Vec2>> split_tmp_buffer_;

  // Cached control-polygon length; updated in pathCallback, used every update().
  double cached_approx_length_{0.0};

  // Current speed command state.
  double current_speed_cmd_{0.0};
  bool have_speed_cmd_{false};

  rclcpp::Subscription<qcar2_msgs::msg::BezierCurve>::SharedPtr sub_path_;
  rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr sub_motor_cmd_;
  rclcpp::Publisher<qcar2_msgs::msg::LateralGuidance>::SharedPtr pub_guidance_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFGGuidanceNode>());
  rclcpp::shutdown();
  return 0;
}

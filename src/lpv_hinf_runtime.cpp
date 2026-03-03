#include "qcar2_control/lpv_hinf_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/LU>

namespace qcar2_control::hinf
{

bool LPVHinfRuntime::buildDiscreteVertex(
  const VertexContinuous & c,
  const double dt,
  DiscreteVertex & out,
  std::string & error) const
{
  if (c.A.empty() || c.B.empty() || c.C.empty() || c.D.empty()) {
    error = "Empty A/B/C/D in vertex data";
    return false;
  }

  const int n = static_cast<int>(c.A.size());
  const int m = static_cast<int>(c.B.front().size());
  if (n <= 0 || m <= 0) {
    error = "Invalid matrix dimensions";
    return false;
  }

  Eigen::MatrixXd A(n, n);
  Eigen::MatrixXd B(n, m);
  Eigen::MatrixXd C(1, n);
  Eigen::MatrixXd D(1, m);

  for (int r = 0; r < n; ++r) {
    if (static_cast<int>(c.A[r].size()) != n || static_cast<int>(c.B[r].size()) != m) {
      error = "Inconsistent A/B row dimensions";
      return false;
    }
    for (int col = 0; col < n; ++col) {
      A(r, col) = c.A[r][col];
    }
    for (int col = 0; col < m; ++col) {
      B(r, col) = c.B[r][col];
    }
  }

  if (static_cast<int>(c.C.size()) != n || static_cast<int>(c.D.size()) != m) {
    error = "Inconsistent C/D dimensions";
    return false;
  }

  for (int col = 0; col < n; ++col) {
    C(0, col) = c.C[col];
  }
  for (int col = 0; col < m; ++col) {
    D(0, col) = c.D[col];
  }

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  const Eigen::MatrixXd M = I - 0.5 * dt * A;

  Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
  if (!lu.isInvertible()) {
    error = "Tustin discretization failed: singular (I - 0.5*dt*A)";
    return false;
  }

  const Eigen::MatrixXd Minv = lu.inverse();

  out.Ad = Minv * (I + 0.5 * dt * A);
  out.Bd = Minv * (dt * B);
  out.Cd = C * Minv;
  out.Dd = D + (C * Minv * (0.5 * dt * B));
  out.x = Eigen::VectorXd::Zero(n);

  return true;
}

bool LPVHinfRuntime::initialize(double dt, const ControllerSet & data, std::string & error)
{
  ready_ = false;
  vertices_.clear();
  discrete_.clear();

  if (dt <= 0.0 || !std::isfinite(dt)) {
    error = "Invalid dt";
    return false;
  }

  if (data.vertices.size() < 2 || data.controllers.size() < 2) {
    error = "Need at least 2 vertices/controllers";
    return false;
  }

  if (data.vertices.size() != data.controllers.size()) {
    error = "vertices/controllers size mismatch";
    return false;
  }

  dt_ = dt;
  vertices_ = data.vertices;
  discrete_.reserve(data.controllers.size());

  for (const auto & vc : data.controllers) {
    DiscreteVertex dv;
    if (!buildDiscreteVertex(vc, dt_, dv, error)) {
      return false;
    }
    discrete_.push_back(std::move(dv));
  }

  ready_ = true;
  return true;
}

void LPVHinfRuntime::reset()
{
  for (auto & d : discrete_) {
    d.x.setZero();
  }
}

double LPVHinfRuntime::compute(const double e_psi, const double delta_meas, double rho)
{
  if (!ready_ || vertices_.size() < 2 || discrete_.size() < 2) {
    return 0.0;
  }

  rho = clamp(rho, vertices_.front(), vertices_.back());

  auto upper = std::upper_bound(vertices_.begin(), vertices_.end(), rho);
  std::size_t idx = 0;
  if (upper == vertices_.begin()) {
    idx = 0;
  } else if (upper == vertices_.end()) {
    idx = vertices_.size() - 2;
  } else {
    idx = static_cast<std::size_t>(std::distance(vertices_.begin(), upper) - 1);
  }

  const double rho_lo = vertices_[idx];
  const double rho_hi = vertices_[idx + 1];
  double alpha = 0.0;
  if (std::fabs(rho_hi - rho_lo) > 1e-12) {
    alpha = (rho - rho_lo) / (rho_hi - rho_lo);
  }

  Eigen::Vector2d y;
  y << -e_psi, delta_meas;

  const auto step_vertex = [&y](DiscreteVertex & d) {
      const double u = (d.Cd * d.x + d.Dd * y)(0, 0);
      d.x = d.Ad * d.x + d.Bd * y;
      return u;
    };

  double u_lo = step_vertex(discrete_[idx]);
  double u_hi = step_vertex(discrete_[idx + 1]);

  return (1.0 - alpha) * u_lo + alpha * u_hi;
}

}  // namespace qcar2_control::hinf

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "qcar2_control/lpv_hinf_v3_data.hpp"

namespace qcar2_control::hinf
{

class LPVHinfRuntime
{
public:
  LPVHinfRuntime() = default;

  bool initialize(double dt, const ControllerSet & data, std::string & error);
  void reset();

  bool ready() const { return ready_; }
  std::size_t vertexCount() const { return vertices_.size(); }

  double compute(double e_psi, double delta_meas, double rho);

private:
  struct DiscreteVertex
  {
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd Cd;
    Eigen::MatrixXd Dd;
    Eigen::VectorXd x;
  };

  bool buildDiscreteVertex(
    const VertexContinuous & c,
    double dt,
    DiscreteVertex & out,
    std::string & error) const;

  static double clamp(double v, double lo, double hi)
  {
    if (v < lo) {
      return lo;
    }
    if (v > hi) {
      return hi;
    }
    return v;
  }

  bool ready_{false};
  double dt_{0.01};
  std::vector<double> vertices_;
  std::vector<DiscreteVertex> discrete_;
};

}  // namespace qcar2_control::hinf

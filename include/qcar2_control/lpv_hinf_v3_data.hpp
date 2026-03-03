#pragma once

#include <vector>

namespace qcar2_control::hinf
{

struct VertexContinuous
{
  std::vector<std::vector<double>> A;
  std::vector<std::vector<double>> B;
  std::vector<double> C;
  std::vector<double> D;
};

struct ControllerSet
{
  std::vector<double> vertices;
  std::vector<VertexContinuous> controllers;
};

const ControllerSet & getLpvHinfV3Data();

}  // namespace qcar2_control::hinf

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// ──────────────────────────────────────────────────────────────────────────────
// 2-D vector type and arithmetic operators
// ──────────────────────────────────────────────────────────────────────────────

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline Vec2 operator+(const Vec2 & a, const Vec2 & b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(const Vec2 & a, const Vec2 & b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(double s, const Vec2 & v) { return {s * v.x, s * v.y}; }

inline double dot(const Vec2 & a, const Vec2 & b) { return a.x * b.x + a.y * b.y; }

inline double norm(const Vec2 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

inline Vec2 normalize(const Vec2 & v)
{
  const double n = norm(v);
  if (n < 1e-9) return {0.0, 0.0};
  return {v.x / n, v.y / n};
}

inline double clamp(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline double wrapToPi(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

// ──────────────────────────────────────────────────────────────────────────────
// Bezier constants
// ──────────────────────────────────────────────────────────────────────────────

static constexpr int kMaxBezierDegree = 64;

// ──────────────────────────────────────────────────────────────────────────────
// Binomial coefficients
// ──────────────────────────────────────────────────────────────────────────────

inline std::vector<double> computeBinom(int n)
{
  std::vector<double> coeffs(static_cast<size_t>(n) + 1u, 1.0);
  for (int k = 1; k < n; ++k) {
    coeffs[static_cast<size_t>(k)] =
      coeffs[static_cast<size_t>(k - 1)] * (static_cast<double>(n - k + 1) / k);
  }
  return coeffs;
}

// ──────────────────────────────────────────────────────────────────────────────
// Bezier curve evaluation (free functions, degree-agnostic)
//
// All three functions accept explicit pts/binom/degree arguments so they can be
// shared between nodes that hold control-point data in different members.
// ──────────────────────────────────────────────────────────────────────────────

inline Vec2 evalBezier(
  double u,
  const std::vector<Vec2> & pts,
  const std::vector<double> & binom,
  int degree)
{
  u = clamp(u, 0.0, 1.0);
  const double one_u = 1.0 - u;
  Vec2 out{0.0, 0.0};

  double u_pows[kMaxBezierDegree + 1];
  double one_u_pows[kMaxBezierDegree + 1];
  u_pows[0] = 1.0;
  one_u_pows[0] = 1.0;
  for (int i = 1; i <= degree; ++i) {
    u_pows[i] = u_pows[i - 1] * u;
    one_u_pows[i] = one_u_pows[i - 1] * one_u;
  }

  for (int i = 0; i <= degree; ++i) {
    const double b = binom[static_cast<size_t>(i)] *
      one_u_pows[degree - i] * u_pows[i];
    out.x += b * pts[static_cast<size_t>(i)].x;
    out.y += b * pts[static_cast<size_t>(i)].y;
  }
  return out;
}

inline Vec2 evalBezierDerivative(
  double u,
  const std::vector<Vec2> & pts,
  const std::vector<double> & binom_d,
  int degree)
{
  if (degree < 1) return {0.0, 0.0};

  u = clamp(u, 0.0, 1.0);
  const double one_u = 1.0 - u;
  Vec2 sum{0.0, 0.0};

  const int d_degree = degree - 1;
  double u_pows[kMaxBezierDegree + 1];
  double one_u_pows[kMaxBezierDegree + 1];
  u_pows[0] = 1.0;
  one_u_pows[0] = 1.0;
  for (int i = 1; i <= d_degree; ++i) {
    u_pows[i] = u_pows[i - 1] * u;
    one_u_pows[i] = one_u_pows[i - 1] * one_u;
  }

  for (int i = 0; i <= d_degree; ++i) {
    const Vec2 d = pts[static_cast<size_t>(i + 1)] - pts[static_cast<size_t>(i)];
    const double b = binom_d[static_cast<size_t>(i)] *
      one_u_pows[d_degree - i] * u_pows[i];
    sum.x += b * d.x;
    sum.y += b * d.y;
  }
  return static_cast<double>(degree) * sum;
}

inline Vec2 evalBezierSecondDerivative(
  double u,
  const std::vector<Vec2> & pts,
  const std::vector<double> & binom_dd,
  int degree)
{
  if (degree < 2) return {0.0, 0.0};

  u = clamp(u, 0.0, 1.0);
  const double one_u = 1.0 - u;
  Vec2 sum{0.0, 0.0};

  const int dd_degree = degree - 2;
  double u_pows[kMaxBezierDegree + 1];
  double one_u_pows[kMaxBezierDegree + 1];
  u_pows[0] = 1.0;
  one_u_pows[0] = 1.0;
  for (int i = 1; i <= dd_degree; ++i) {
    u_pows[i] = u_pows[i - 1] * u;
    one_u_pows[i] = one_u_pows[i - 1] * one_u;
  }

  for (int i = 0; i <= dd_degree; ++i) {
    const Vec2 dd =
      pts[static_cast<size_t>(i + 2)] -
      2.0 * pts[static_cast<size_t>(i + 1)] +
      pts[static_cast<size_t>(i)];
    const double b = binom_dd[static_cast<size_t>(i)] *
      one_u_pows[dd_degree - i] * u_pows[i];
    sum.x += b * dd.x;
    sum.y += b * dd.y;
  }
  return static_cast<double>(degree * (degree - 1)) * sum;
}

// ──────────────────────────────────────────────────────────────────────────────
// Curve geometry helpers
// ──────────────────────────────────────────────────────────────────────────────

inline double pointLineDistance(const Vec2 & p, const Vec2 & a, const Vec2 & b)
{
  const Vec2 ab = b - a;
  const double ab_norm = norm(ab);
  if (ab_norm < 1e-12) return norm(p - a);
  const double area = std::fabs((p.x - a.x) * ab.y - (p.y - a.y) * ab.x);
  return area / ab_norm;
}

inline Vec2 closestPointOnSegment(
  const Vec2 & p, const Vec2 & a, const Vec2 & b, double & t_out)
{
  const Vec2 ab = b - a;
  const double denom = dot(ab, ab);
  if (denom < 1e-12) { t_out = 0.0; return a; }
  double t = dot(p - a, ab) / denom;
  t = clamp(t, 0.0, 1.0);
  t_out = t;
  return a + t * ab;
}

inline void deCasteljauSplit(
  const std::vector<Vec2> & points,
  std::vector<Vec2> & left,
  std::vector<Vec2> & right,
  std::vector<Vec2> & tmp)
{
  const size_t n = points.size();
  left.resize(n);
  right.resize(n);
  tmp.resize(n);

  for (size_t i = 0; i < n; ++i) tmp[i] = points[i];

  left[0] = tmp[0];
  right[n - 1] = tmp[n - 1];

  for (size_t level = 1; level < n; ++level) {
    for (size_t i = 0; i < n - level; ++i) {
      tmp[i] = 0.5 * tmp[i] + 0.5 * tmp[i + 1];
    }
    left[level] = tmp[0];
    right[n - 1 - level] = tmp[n - level - 1];
  }
}

inline double controlPolygonFlatness(const std::vector<Vec2> & points)
{
  if (points.size() <= 2) return 0.0;
  const Vec2 & a = points.front();
  const Vec2 & b = points.back();
  double max_dist = 0.0;
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    max_dist = std::max(max_dist, pointLineDistance(points[i], a, b));
  }
  return max_dist;
}

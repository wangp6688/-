/**
 * CurveVectorTransport.h
 *
 * Parallel transport of direction vectors along a discrete 3-D curve
 * (rotation-minimising / Bishop-frame propagation).
 *
 * Problem
 * -------
 * Given:
 *   - An ordered 3-D point set that defines a curve (polyline).
 *   - A **start point** on the curve and a set of **direction vectors** that
 *     all lie in the plane whose normal is the curve tangent at that point
 *     (i.e. they are all perpendicular to the local tangent).
 *   - An **end point** on the curve.
 *
 * Compute:
 *   - The direction vectors "projected" (transported) from the start to the
 *     end along the curve, staying perpendicular to the tangent at every step.
 *
 * Method
 * ------
 * At each edge of the polyline the frame is rotated by the smallest rotation
 * that maps the current edge tangent onto the next one (Rodrigues' formula).
 * This is the discrete analogue of parallel transport and produces a
 * rotation-minimising (Bishop) frame with no unnecessary twist.
 *
 * Quick-start
 * -----------
 * @code
 *   #include "CurveVectorTransport.h"
 *
 *   using Vec3 = CurveVectorTransport::Vec3;
 *
 *   // Build the curve point set.
 *   std::vector<Vec3> pts = { {0,0,0}, {1,0,0}, {2,0.5,0}, {3,1,0} };
 *
 *   // Direction vectors at the start (perpendicular to the tangent there).
 *   std::vector<Vec3> dirs = { {0,1,0}, {0,0,1} };
 *
 *   CurveVectorTransport cvt;
 *   cvt.SetCurvePoints(pts);
 *   cvt.SetStartPoint({0,0,0});
 *   cvt.SetDirections(dirs);
 *
 *   auto result = cvt.TransportToEndPoint({3,1,0});
 *   // result[0] and result[1] are the transported vectors at the end point.
 * @endcode
 *
 * Notes
 * -----
 * - The start / end points are matched to the **nearest** point in the set.
 * - Any component of a direction vector along the tangent is projected out
 *   before transport begins, enforcing the perpendicularity constraint.
 * - Transport can proceed in either direction along the curve (start index
 *   greater or smaller than end index).
 * - No external dependencies – standard C++14 only.
 */

#ifndef CURVE_VECTOR_TRANSPORT_H
#define CURVE_VECTOR_TRANSPORT_H

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

class CurveVectorTransport
{
public:
  // ── Public types ───────────────────────────────────────────────────────────

  /** A 3-D vector / point represented as a plain array. */
  using Vec3 = std::array<double, 3>;

  // ── Setters ────────────────────────────────────────────────────────────────

  /** Set the ordered point set that defines the curve. */
  void SetCurvePoints(const std::vector<Vec3>& pts) { m_pts = pts; }

  /** Set the start point on the curve. */
  void SetStartPoint(const Vec3& pt) { m_start = pt; }

  /**
   * Set the direction vectors at the start point.
   *
   * They are expected to be perpendicular to the curve tangent at the start,
   * but this is not strictly required: any component along the tangent is
   * automatically removed before the transport begins.
   */
  void SetDirections(const std::vector<Vec3>& dirs) { m_dirs = dirs; }

  // ── Main computation ───────────────────────────────────────────────────────

  /**
   * Transport all stored direction vectors from the start point to @p endPoint
   * along the curve and return the resulting vectors at @p endPoint.
   *
   * @param endPoint  Target point (matched to the nearest curve point).
   * @return          Transported direction vectors, one per input direction.
   *
   * @throws std::runtime_error  If the curve has fewer than 2 points or if
   *                             no direction vectors have been set.
   */
  std::vector<Vec3> TransportToEndPoint(const Vec3& endPoint) const
  {
    if (m_pts.size() < 2)
      throw std::runtime_error(
        "CurveVectorTransport: curve must have at least 2 points.");
    if (m_dirs.empty())
      throw std::runtime_error(
        "CurveVectorTransport: no direction vectors have been set.");

    // 1. Locate start and end indices (nearest-point search).
    const int si = NearestIndex(m_start);
    const int ei = NearestIndex(endPoint);

    // 2. Compute the initial tangent in the direction of travel so it is
    //    consistent with the first edge direction used in the walk below.
    //    (Using the bi-directional TangentAt would give a forward-pointing
    //    tangent even when we travel backward, causing a spurious 180° flip.)
    const int step = (ei >= si) ? +1 : -1;
    Vec3 T = TravelTangentAt(si, step);

    // 3. Project the input directions onto the plane perpendicular to T.
    std::vector<Vec3> vecs(m_dirs.size());
    for (std::size_t k = 0; k < m_dirs.size(); ++k)
      vecs[k] = ProjectOntoPlane(m_dirs[k], T);

    // 4. Walk from si to ei one edge at a time, rotating the frame at each step.
    for (int i = si; i != ei; i += step)
    {
      // Edge tangent in the direction of travel.
      Vec3 T_next = Normalize(Sub(m_pts[i + step], m_pts[i]));

      // Apply the rotation that maps T onto T_next to every direction vector.
      for (auto& v : vecs)
        v = RotateByTangentChange(v, T, T_next);

      T = T_next;
    }

    return vecs;
  }

private:
  // ── Data members ──────────────────────────────────────────────────────────

  std::vector<Vec3> m_pts;
  Vec3              m_start{};
  std::vector<Vec3> m_dirs;

  // ── Vector math helpers ───────────────────────────────────────────────────

  static double Dot(const Vec3& a, const Vec3& b)
  {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  }

  static Vec3 Cross(const Vec3& a, const Vec3& b)
  {
    return { a[1]*b[2] - a[2]*b[1],
             a[2]*b[0] - a[0]*b[2],
             a[0]*b[1] - a[1]*b[0] };
  }

  static Vec3 Scale(const Vec3& a, double s)
  {
    return { a[0]*s, a[1]*s, a[2]*s };
  }

  static Vec3 Add(const Vec3& a, const Vec3& b)
  {
    return { a[0]+b[0], a[1]+b[1], a[2]+b[2] };
  }

  static Vec3 Sub(const Vec3& a, const Vec3& b)
  {
    return { a[0]-b[0], a[1]-b[1], a[2]-b[2] };
  }

  static double Norm(const Vec3& a) { return std::sqrt(Dot(a, a)); }

  static Vec3 Normalize(const Vec3& a)
  {
    const double n = Norm(a);
    return (n > 1e-15) ? Scale(a, 1.0 / n) : Vec3{0, 0, 0};
  }

  /**
   * Project v onto the plane with unit normal n:
   *   v_proj = v - (v · n) * n
   */
  static Vec3 ProjectOntoPlane(const Vec3& v, const Vec3& n)
  {
    return Sub(v, Scale(n, Dot(v, n)));
  }

  /**
   * Return an arbitrary unit vector perpendicular to the given unit vector t.
   * Used only for the degenerate anti-parallel case.
   */
  static Vec3 Perpendicular(const Vec3& t)
  {
    // Pick the global axis least aligned with t, then orthogonalise.
    const Vec3 candidate =
      (std::abs(t[0]) < std::abs(t[1]) && std::abs(t[0]) < std::abs(t[2]))
        ? Vec3{1, 0, 0}
        : (std::abs(t[1]) < std::abs(t[2]) ? Vec3{0, 1, 0} : Vec3{0, 0, 1});
    return Normalize(Cross(t, candidate));
  }

  /**
   * Rotate vector @p v by the rotation that maps unit vector @p T0 onto
   * unit vector @p T1, using Rodrigues' formula.
   *
   * Three cases:
   *  - T0 ≈ T1   (same direction):   identity, return v unchanged.
   *  - T0 ≈ -T1  (anti-parallel):    180° rotation around a perpendicular axis.
   *  - General:  rotate around (T0 × T1) by arccos(T0 · T1).
   */
  static Vec3 RotateByTangentChange(const Vec3& v,
                                    const Vec3& T0,
                                    const Vec3& T1)
  {
    const Vec3   axis = Cross(T0, T1);
    const double sinA = Norm(axis);   // sin of the rotation angle
    const double cosA = Dot(T0, T1); // cos of the rotation angle

    // Nearly identical tangents — no meaningful rotation.
    if (sinA < 1e-10)
    {
      if (cosA >= 0.0)
        return v; // same direction, identity

      // Anti-parallel tangents: 180-degree rotation around any perpendicular axis.
      const Vec3 perp = Perpendicular(T0);
      // Rodrigues at theta = pi: v_rot = 2*(perp · v)*perp - v
      return Sub(Scale(perp, 2.0 * Dot(perp, v)), v);
    }

    // General case — Rodrigues' formula:
    //   v_rot = v*cos(θ) + (k × v)*sin(θ) + k*(k · v)*(1 − cos(θ))
    const Vec3   k       = Scale(axis, 1.0 / sinA); // normalised rotation axis
    const double kDotV   = Dot(k, v);
    const Vec3   kCrossV = Cross(k, v);

    return Add(Add(Scale(v, cosA),
                   Scale(kCrossV, sinA)),
               Scale(k, kDotV * (1.0 - cosA)));
  }

  // ── Curve helpers ─────────────────────────────────────────────────────────

  /** Return the index of the curve point nearest to @p pt. */
  int NearestIndex(const Vec3& pt) const
  {
    int    best  = 0;
    double bestD = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(m_pts.size()); ++i)
    {
      const Vec3   d  = Sub(m_pts[i], pt);
      const double d2 = Dot(d, d);
      if (d2 < bestD) { bestD = d2; best = i; }
    }
    return best;
  }

  /**
   * Tangent at index @p idx pointing in the direction of travel (@p step = ±1).
   *
   * For a forward walk  (step = +1) this is the forward-difference tangent.
   * For a backward walk (step = -1) this is the backward-difference tangent
   * (i.e. it points in the −x direction of the curve).
   * This ensures the initial tangent is always consistent with the first
   * edge direction used in the transport loop.
   */
  Vec3 TravelTangentAt(int idx, int step) const
  {
    const int n = static_cast<int>(m_pts.size());
    if (step >= 0)
    {
      // Forward tangent: edge idx → idx+1.
      if (idx == n - 1) return Normalize(Sub(m_pts[n-1], m_pts[n-2]));
      return Normalize(Sub(m_pts[idx + 1], m_pts[idx]));
    }
    else
    {
      // Backward tangent: edge idx → idx-1 (points toward lower indices).
      if (idx == 0) return Normalize(Sub(m_pts[0], m_pts[1]));
      return Normalize(Sub(m_pts[idx - 1], m_pts[idx]));
    }
  }
};

#endif // CURVE_VECTOR_TRANSPORT_H

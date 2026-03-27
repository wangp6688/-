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
 *   - The direction vectors transported from the start to the end along the
 *     curve, staying perpendicular to the tangent at every step.
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
 *   std::vector<vtkVector3d> pts  = { {0,0,0}, {1,0,0}, {2,0.5,0}, {3,1,0} };
 *   std::vector<vtkVector3d> dirs = { {0,1,0}, {0,0,1} };
 *
 *   auto result = TransportVectorsAlongCurve(pts, {0,0,0}, dirs, {3,1,0});
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
 * - Requires VTK (vtkMath, vtkVector).
 */

#ifndef CURVE_VECTOR_TRANSPORT_H
#define CURVE_VECTOR_TRANSPORT_H

#include <vtkMath.h>
#include <vtkVector.h>

#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

// ── Implementation details (not part of the public API) ───────────────────────
namespace CurveVectorTransportDetail
{

/** Normalize @p v and return the result (does not modify @p v). */
inline vtkVector3d Normalized(vtkVector3d v)
{
  const double n = vtkMath::Norm(v.GetData());
  if (n > 1e-15)
    vtkMath::Normalize(v.GetData());
  return v;
}

/** Project @p v onto the plane with unit normal @p n: v - (v·n)*n */
inline vtkVector3d ProjectOntoPlane(const vtkVector3d& v, const vtkVector3d& n)
{
  return v - n * vtkMath::Dot(v.GetData(), n.GetData());
}

/**
 * Return an arbitrary unit vector perpendicular to unit vector @p t.
 * Used only for the degenerate anti-parallel case.
 */
inline vtkVector3d Perpendicular(const vtkVector3d& t)
{
  // Pick the global axis least aligned with t, then orthogonalise.
  vtkVector3d candidate;
  if (std::abs(t[0]) < std::abs(t[1]) && std::abs(t[0]) < std::abs(t[2]))
    candidate = vtkVector3d(1, 0, 0);
  else if (std::abs(t[1]) < std::abs(t[2]))
    candidate = vtkVector3d(0, 1, 0);
  else
    candidate = vtkVector3d(0, 0, 1);

  vtkVector3d result;
  vtkMath::Cross(t.GetData(), candidate.GetData(), result.GetData());
  return Normalized(result);
}

/**
 * Rotate @p v by the rotation that maps unit vector @p T0 onto unit vector
 * @p T1, using Rodrigues' formula.
 *
 * Three cases:
 *  - T0 ≈ T1   (same direction):   identity, return v unchanged.
 *  - T0 ≈ -T1  (anti-parallel):    180° rotation around a perpendicular axis.
 *  - General:  rotate around (T0 × T1) by arccos(T0 · T1).
 */
inline vtkVector3d RotateByTangentChange(const vtkVector3d& v,
                                         const vtkVector3d& T0,
                                         const vtkVector3d& T1)
{
  vtkVector3d axis;
  vtkMath::Cross(T0.GetData(), T1.GetData(), axis.GetData());

  const double sinA = vtkMath::Norm(axis.GetData()); // sin of the rotation angle
  const double cosA = vtkMath::Dot(T0.GetData(), T1.GetData()); // cos

  // Nearly identical tangents — no meaningful rotation.
  if (sinA < 1e-10)
  {
    if (cosA >= 0.0)
      return v; // same direction, identity

    // Anti-parallel tangents: 180-degree rotation around any perpendicular axis.
    const vtkVector3d perp = Perpendicular(T0);
    // Rodrigues at theta = pi: v_rot = 2*(perp·v)*perp - v
    return perp * (2.0 * vtkMath::Dot(perp.GetData(), v.GetData())) - v;
  }

  // General case — Rodrigues' formula:
  //   v_rot = v*cos(θ) + (k × v)*sin(θ) + k*(k · v)*(1 − cos(θ))
  const vtkVector3d k = axis * (1.0 / sinA); // normalised rotation axis

  vtkVector3d kCrossV;
  vtkMath::Cross(k.GetData(), v.GetData(), kCrossV.GetData());

  const double kDotV = vtkMath::Dot(k.GetData(), v.GetData());

  return v * cosA + kCrossV * sinA + k * (kDotV * (1.0 - cosA));
}

/**
 * Return the index of the curve point nearest to @p pt using
 * vtkMath::Distance2BetweenPoints.
 */
inline int NearestIndex(const std::vector<vtkVector3d>& pts,
                        const vtkVector3d& pt)
{
  int    best  = 0;
  double bestD = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
  {
    const double d2 =
      vtkMath::Distance2BetweenPoints(pts[i].GetData(), pt.GetData());
    if (d2 < bestD) { bestD = d2; best = i; }
  }
  return best;
}

/**
 * Tangent at index @p idx pointing in the direction of travel (@p step = ±1).
 *
 * For a forward walk  (step = +1) this is the forward-difference tangent.
 * For a backward walk (step = -1) this is the backward-difference tangent.
 * This ensures the initial tangent is always consistent with the first
 * edge direction used in the transport loop.
 */
inline vtkVector3d TravelTangentAt(const std::vector<vtkVector3d>& pts,
                                   int idx, int step)
{
  const int n = static_cast<int>(pts.size());
  if (step >= 0)
  {
    // Forward tangent: edge idx → idx+1.
    if (idx == n - 1) return Normalized(pts[n-1] - pts[n-2]);
    return Normalized(pts[idx + 1] - pts[idx]);
  }
  else
  {
    // Backward tangent: edge idx → idx-1 (points toward lower indices).
    if (idx == 0) return Normalized(pts[0] - pts[1]);
    return Normalized(pts[idx - 1] - pts[idx]);
  }
}

} // namespace CurveVectorTransportDetail

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Transport a set of direction vectors from @p startPoint to @p endPoint
 * along the discrete 3-D curve defined by @p curvePoints.
 *
 * @param curvePoints  Ordered 3-D polyline points (at least 2).
 * @param startPoint   Origin of the transport (matched to nearest curve point).
 * @param directions   Direction vectors at the start, expected to be
 *                     perpendicular to the curve tangent there. Any tangential
 *                     component is automatically projected out.
 * @param endPoint     Target point (matched to nearest curve point).
 * @return             Transported direction vectors, one per input direction.
 *
 * @throws std::invalid_argument  If fewer than 2 curve points are provided or
 *                                if @p directions is empty.
 */
inline std::vector<vtkVector3d> TransportVectorsAlongCurve(
  const std::vector<vtkVector3d>& curvePoints,
  const vtkVector3d&              startPoint,
  const std::vector<vtkVector3d>& directions,
  const vtkVector3d&              endPoint)
{
  using namespace CurveVectorTransportDetail;

  if (curvePoints.size() < 2)
    throw std::invalid_argument(
      "TransportVectorsAlongCurve: curvePoints must have at least 2 points.");
  if (directions.empty())
    throw std::invalid_argument(
      "TransportVectorsAlongCurve: directions must not be empty.");

  // 1. Locate start and end indices (nearest-point search).
  const int si = NearestIndex(curvePoints, startPoint);
  const int ei = NearestIndex(curvePoints, endPoint);

  // 2. Compute the initial tangent in the direction of travel so it is
  //    consistent with the first edge direction used in the walk below.
  //    (A bidirectional tangent would cause a spurious 180° flip when
  //    walking backward.)
  const int step = (ei >= si) ? +1 : -1;
  vtkVector3d T = TravelTangentAt(curvePoints, si, step);

  // 3. Project the input directions onto the plane perpendicular to T.
  std::vector<vtkVector3d> vecs(directions.size());
  for (std::size_t k = 0; k < directions.size(); ++k)
    vecs[k] = ProjectOntoPlane(directions[k], T);

  // 4. Walk from si to ei one edge at a time, rotating the frame at each step.
  for (int i = si; i != ei; i += step)
  {
    // Edge tangent in the direction of travel.
    vtkVector3d T_next = Normalized(curvePoints[i + step] - curvePoints[i]);

    // Apply the rotation that maps T onto T_next to every direction vector.
    for (auto& v : vecs)
      v = RotateByTangentChange(v, T, T_next);

    T = T_next;
  }

  return vecs;
}

#endif // CURVE_VECTOR_TRANSPORT_H

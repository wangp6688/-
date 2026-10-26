#include "vtkTrajectoryRoundCapSource.h"

#include <cmath>

#include <vtkCellArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

vtkStandardNewMacro(vtkTrajectoryRoundCapSource);

namespace
{
constexpr double EPS = 1e-12;

inline bool Normalize3(double v[3])
{
  return vtkMath::Normalize(v) > EPS;
}

inline std::array<double, 3> ToArray(const double p[3])
{
  return { p[0], p[1], p[2] };
}

inline void Sub3(const std::array<double, 3>& a, const std::array<double, 3>& b, double out[3])
{
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

inline void AddScaled3(const std::array<double, 3>& p, const double v[3], double s, double out[3])
{
  out[0] = p[0] + v[0] * s;
  out[1] = p[1] + v[1] * s;
  out[2] = p[2] + v[2] * s;
}
}

vtkTrajectoryRoundCapSource::vtkTrajectoryRoundCapSource()
{
  this->Normal[0] = 0.0;
  this->Normal[1] = 0.0;
  this->Normal[2] = 1.0;
  this->Radius = 1.0;
  this->CapResolution = 16;
  this->SetNumberOfInputPorts(0);
}

void vtkTrajectoryRoundCapSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Normal: (" << this->Normal[0] << ", " << this->Normal[1] << ", " << this->Normal[2]
     << ")\n";
  os << indent << "Radius: " << this->Radius << "\n";
  os << indent << "CapResolution: " << this->CapResolution << "\n";
  os << indent << "TrajectoryPointCount: " << this->TrajectoryPoints.size() << "\n";
}

void vtkTrajectoryRoundCapSource::ClearPoints()
{
  if (!this->TrajectoryPoints.empty())
  {
    this->TrajectoryPoints.clear();
    this->Modified();
  }
}

void vtkTrajectoryRoundCapSource::AddPoint(double x, double y, double z)
{
  this->TrajectoryPoints.push_back({ x, y, z });
  this->Modified();
}

void vtkTrajectoryRoundCapSource::SetPoints(vtkPoints* points)
{
  this->TrajectoryPoints.clear();
  if (points)
  {
    this->TrajectoryPoints.reserve(static_cast<size_t>(points->GetNumberOfPoints()));
    double p[3];
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
    {
      points->GetPoint(i, p);
      this->TrajectoryPoints.push_back(ToArray(p));
    }
  }
  this->Modified();
}

vtkIdType vtkTrajectoryRoundCapSource::GetNumberOfTrajectoryPoints() const
{
  return static_cast<vtkIdType>(this->TrajectoryPoints.size());
}

int vtkTrajectoryRoundCapSource::RequestData(
  vtkInformation*,
  vtkInformationVector**,
  vtkInformationVector* outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);
  output->Initialize();

  if (this->TrajectoryPoints.empty())
  {
    return 1;
  }

  double n[3] = { this->Normal[0], this->Normal[1], this->Normal[2] };
  if (!Normalize3(n))
  {
    vtkErrorMacro("Normal vector has zero length.");
    return 0;
  }

  std::vector<std::array<double, 3>> points;
  points.reserve(this->TrajectoryPoints.size());
  points.push_back(this->TrajectoryPoints.front());
  for (size_t i = 1; i < this->TrajectoryPoints.size(); ++i)
  {
    double d[3];
    Sub3(this->TrajectoryPoints[i], points.back(), d);
    if (vtkMath::Norm(d) > EPS)
    {
      points.push_back(this->TrajectoryPoints[i]);
    }
  }

  if (points.empty())
  {
    return 1;
  }

  vtkNew<vtkPoints> outPts;
  outPts->SetDataTypeToDouble();
  vtkNew<vtkCellArray> polys;

  const double radius = this->Radius;
  const int capRes = this->CapResolution;

  if (points.size() == 1)
  {
    const int circleRes = capRes * 2;
    vtkIdType centerId = outPts->InsertNextPoint(points[0].data());
    std::vector<vtkIdType> ringIds;
    ringIds.reserve(static_cast<size_t>(circleRes));
    for (int i = 0; i < circleRes; ++i)
    {
      const double theta = (2.0 * vtkMath::Pi() * static_cast<double>(i)) / static_cast<double>(circleRes);
      double tangent[3] = { std::cos(theta), std::sin(theta), 0.0 };
      double dir[3];
      vtkMath::Cross(n, tangent, dir);
      if (!Normalize3(dir))
      {
        continue;
      }
      double p[3];
      AddScaled3(points[0], dir, radius, p);
      ringIds.push_back(outPts->InsertNextPoint(p));
    }
    for (size_t i = 0; i < ringIds.size(); ++i)
    {
      vtkIdType tri[3] = { centerId, ringIds[i], ringIds[(i + 1) % ringIds.size()] };
      polys->InsertNextCell(3, tri);
    }
    output->SetPoints(outPts);
    output->SetPolys(polys);
    return 1;
  }

  const size_t nPts = points.size();
  std::vector<std::array<double, 3>> tangents(nPts);
  for (size_t i = 0; i < nPts; ++i)
  {
    double t[3] = { 0.0, 0.0, 0.0 };
    if (i == 0)
    {
      Sub3(points[1], points[0], t);
    }
    else if (i + 1 == nPts)
    {
      Sub3(points[nPts - 1], points[nPts - 2], t);
    }
    else
    {
      double t0[3], t1[3];
      Sub3(points[i], points[i - 1], t0);
      Sub3(points[i + 1], points[i], t1);
      Normalize3(t0);
      Normalize3(t1);
      t[0] = t0[0] + t1[0];
      t[1] = t0[1] + t1[1];
      t[2] = t0[2] + t1[2];
      if (!Normalize3(t))
      {
        t[0] = t1[0];
        t[1] = t1[1];
        t[2] = t1[2];
      }
    }
    if (!Normalize3(t))
    {
      vtkErrorMacro("Trajectory contains degenerate segments.");
      return 0;
    }
    tangents[i] = ToArray(t);
  }

  std::vector<vtkIdType> leftIds(nPts);
  std::vector<vtkIdType> rightIds(nPts);
  std::vector<std::array<double, 3>> sides(nPts);
  for (size_t i = 0; i < nPts; ++i)
  {
    const auto& t = tangents[i];
    double side[3];
    vtkMath::Cross(n, t.data(), side);
    if (!Normalize3(side))
    {
      vtkErrorMacro("Normal is parallel to trajectory tangent.");
      return 0;
    }
    sides[i] = ToArray(side);

    double lp[3], rp[3];
    AddScaled3(points[i], side, radius, lp);
    AddScaled3(points[i], side, -radius, rp);
    leftIds[i] = outPts->InsertNextPoint(lp);
    rightIds[i] = outPts->InsertNextPoint(rp);
  }

  for (size_t i = 0; i + 1 < nPts; ++i)
  {
    vtkIdType tri1[3] = { leftIds[i], leftIds[i + 1], rightIds[i] };
    vtkIdType tri2[3] = { rightIds[i], leftIds[i + 1], rightIds[i + 1] };
    polys->InsertNextCell(3, tri1);
    polys->InsertNextCell(3, tri2);
  }

  vtkIdType startCenterId = outPts->InsertNextPoint(points.front().data());
  std::vector<vtkIdType> startArc;
  startArc.reserve(static_cast<size_t>(capRes + 1));
  startArc.push_back(rightIds.front());
  for (int i = 1; i < capRes; ++i)
  {
    const double theta = (vtkMath::Pi() * static_cast<double>(i)) / static_cast<double>(capRes);
    double dir[3];
    dir[0] = -std::cos(theta) * sides.front()[0] - std::sin(theta) * tangents.front()[0];
    dir[1] = -std::cos(theta) * sides.front()[1] - std::sin(theta) * tangents.front()[1];
    dir[2] = -std::cos(theta) * sides.front()[2] - std::sin(theta) * tangents.front()[2];
    double p[3];
    AddScaled3(points.front(), dir, radius, p);
    startArc.push_back(outPts->InsertNextPoint(p));
  }
  startArc.push_back(leftIds.front());
  for (size_t i = 0; i + 1 < startArc.size(); ++i)
  {
    vtkIdType tri[3] = { startCenterId, startArc[i], startArc[i + 1] };
    polys->InsertNextCell(3, tri);
  }

  vtkIdType endCenterId = outPts->InsertNextPoint(points.back().data());
  std::vector<vtkIdType> endArc;
  endArc.reserve(static_cast<size_t>(capRes + 1));
  endArc.push_back(leftIds.back());
  for (int i = 1; i < capRes; ++i)
  {
    const double theta = (vtkMath::Pi() * static_cast<double>(i)) / static_cast<double>(capRes);
    double dir[3];
    dir[0] = std::cos(theta) * sides.back()[0] + std::sin(theta) * tangents.back()[0];
    dir[1] = std::cos(theta) * sides.back()[1] + std::sin(theta) * tangents.back()[1];
    dir[2] = std::cos(theta) * sides.back()[2] + std::sin(theta) * tangents.back()[2];
    double p[3];
    AddScaled3(points.back(), dir, radius, p);
    endArc.push_back(outPts->InsertNextPoint(p));
  }
  endArc.push_back(rightIds.back());
  for (size_t i = 0; i + 1 < endArc.size(); ++i)
  {
    vtkIdType tri[3] = { endCenterId, endArc[i], endArc[i + 1] };
    polys->InsertNextCell(3, tri);
  }

  output->SetPoints(outPts);
  output->SetPolys(polys);
  return 1;
}

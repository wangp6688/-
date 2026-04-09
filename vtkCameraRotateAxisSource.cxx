/**
 * vtkCameraRotateAxisSource.cxx
 *
 * Geometry baked from rotate_axis.svg (viewBox="0 0 323 16").
 * The 2 SVG sub-paths were tessellated into 138 sample points
 * (cubic Bezier curves subdivided into 32 segments each).
 * Coordinates are normalised to the range [-0.5, +0.5] and centred at (0, 0).
 * The SVG Y-axis (pointing down) is flipped so that the local +Y axis points up.
 */

#include "vtkCameraRotateAxisSource.h"

#include <vtkCellArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>

vtkStandardNewMacro(vtkCameraRotateAxisSource);

// ─── Baked geometry ──────────────────────────────────────────────────────────

static const int ContourCount = 2;

static const int ContourPointCounts[ContourCount] = {
  69, 69
};

// Total: 138 points across 2 contours.
// Layout: first 69 points are contour 0; next 69 points are contour 1.
static const double ContourPoints[][2] = {
  // contour 0
  {    -0.46169907,     0.03762375 },
  {    -0.24380402,     0.05897750 },
  {    -0.24380729,     0.05536536 },
  {    -0.24381045,     0.05175250 },
  {    -0.24381351,     0.04813892 },
  {    -0.24381647,     0.04452464 },
  {    -0.24381932,     0.04090965 },
  {    -0.24382207,     0.03729397 },
  {    -0.24382471,     0.03367761 },
  {    -0.24382723,     0.03006057 },
  {    -0.24382965,     0.02644285 },
  {    -0.24383195,     0.02282447 },
  {    -0.24383413,     0.01920543 },
  {    -0.24383619,     0.01558574 },
  {    -0.24383813,     0.01196541 },
  {    -0.24383994,     0.00834444 },
  {    -0.24384163,     0.00472285 },
  {    -0.24384319,     0.00110063 },
  {    -0.24384462,    -0.00252221 },
  {    -0.24384591,    -0.00614565 },
  {    -0.24384708,    -0.00976970 },
  {    -0.24384810,    -0.01339434 },
  {    -0.24384899,    -0.01701956 },
  {    -0.24384973,    -0.02064537 },
  {    -0.24385033,    -0.02427175 },
  {    -0.24385078,    -0.02789869 },
  {    -0.24385109,    -0.03152619 },
  {    -0.24385125,    -0.03515425 },
  {    -0.24385125,    -0.03878284 },
  {    -0.24385110,    -0.04241198 },
  {    -0.24385080,    -0.04604165 },
  {    -0.24385033,    -0.04967183 },
  {    -0.24384970,    -0.05330254 },
  {    -0.24384892,    -0.05693375 },
  {    -0.24384860,    -0.05813453 },
  {    -0.24384825,    -0.05933524 },
  {    -0.24384786,    -0.06053588 },
  {    -0.24384743,    -0.06173645 },
  {    -0.24384696,    -0.06293694 },
  {    -0.24384646,    -0.06413736 },
  {    -0.24384592,    -0.06533771 },
  {    -0.24384536,    -0.06653799 },
  {    -0.24384477,    -0.06773819 },
  {    -0.24384415,    -0.06893833 },
  {    -0.24384350,    -0.07013839 },
  {    -0.24384284,    -0.07133839 },
  {    -0.24384215,    -0.07253831 },
  {    -0.24384144,    -0.07373816 },
  {    -0.24384072,    -0.07493794 },
  {    -0.24383998,    -0.07613766 },
  {    -0.24383922,    -0.07733730 },
  {    -0.24383846,    -0.07853687 },
  {    -0.24383768,    -0.07973638 },
  {    -0.24383690,    -0.08093581 },
  {    -0.24383611,    -0.08213518 },
  {    -0.24383531,    -0.08333448 },
  {    -0.24383452,    -0.08453371 },
  {    -0.24383372,    -0.08573287 },
  {    -0.24383293,    -0.08693197 },
  {    -0.24383213,    -0.08813099 },
  {    -0.24383135,    -0.08932995 },
  {    -0.24383057,    -0.09052885 },
  {    -0.24382980,    -0.09172767 },
  {    -0.24382904,    -0.09292643 },
  {    -0.24382829,    -0.09412512 },
  {    -0.24382755,    -0.09532375 },
  {    -0.46166192,    -0.11667125 },
  {    -0.49978580,    -0.48863750 },
  {    -0.50000000,     0.40209937 },
  // contour 1
  {     0.49889474,    -0.39076875 },
  {     0.46059133,    -0.02629000 },
  {     0.24146749,    -0.04776437 },
  {     0.24146751,    -0.04656563 },
  {     0.24146756,    -0.04536683 },
  {     0.24146764,    -0.04416796 },
  {     0.24146775,    -0.04296903 },
  {     0.24146789,    -0.04177004 },
  {     0.24146804,    -0.04057098 },
  {     0.24146822,    -0.03937186 },
  {     0.24146841,    -0.03817268 },
  {     0.24146862,    -0.03697343 },
  {     0.24146883,    -0.03577411 },
  {     0.24146906,    -0.03457474 },
  {     0.24146929,    -0.03337530 },
  {     0.24146952,    -0.03217579 },
  {     0.24146975,    -0.03097622 },
  {     0.24146998,    -0.02977658 },
  {     0.24147020,    -0.02857687 },
  {     0.24147041,    -0.02737711 },
  {     0.24147061,    -0.02617727 },
  {     0.24147080,    -0.02497737 },
  {     0.24147097,    -0.02377740 },
  {     0.24147112,    -0.02257736 },
  {     0.24147124,    -0.02137726 },
  {     0.24147134,    -0.02017709 },
  {     0.24147141,    -0.01897686 },
  {     0.24147145,    -0.01777655 },
  {     0.24147145,    -0.01657618 },
  {     0.24147142,    -0.01537574 },
  {     0.24147134,    -0.01417523 },
  {     0.24147123,    -0.01297465 },
  {     0.24147106,    -0.01177400 },
  {     0.24147085,    -0.01057329 },
  {     0.24147059,    -0.00937250 },
  {     0.24146965,    -0.00574130 },
  {     0.24146856,    -0.00211063 },
  {     0.24146733,     0.00151949 },
  {     0.24146595,     0.00514906 },
  {     0.24146443,     0.00877808 },
  {     0.24146277,     0.01240654 },
  {     0.24146097,     0.01603442 },
  {     0.24145903,     0.01966173 },
  {     0.24145694,     0.02328845 },
  {     0.24145472,     0.02691458 },
  {     0.24145236,     0.03054011 },
  {     0.24144985,     0.03416503 },
  {     0.24144721,     0.03778934 },
  {     0.24144443,     0.04141303 },
  {     0.24144152,     0.04503609 },
  {     0.24143847,     0.04865852 },
  {     0.24143528,     0.05228030 },
  {     0.24143196,     0.05590143 },
  {     0.24142850,     0.05952190 },
  {     0.24142490,     0.06314172 },
  {     0.24142118,     0.06676085 },
  {     0.24141732,     0.07037931 },
  {     0.24141333,     0.07399709 },
  {     0.24140920,     0.07761417 },
  {     0.24140494,     0.08123055 },
  {     0.24140056,     0.08484622 },
  {     0.24139604,     0.08846118 },
  {     0.24139139,     0.09207542 },
  {     0.24138662,     0.09568893 },
  {     0.24138171,     0.09930170 },
  {     0.24137668,     0.10291372 },
  {     0.24137152,     0.10652500 },
  {     0.46055418,     0.12800500 },
  {     0.49867802,     0.49997079 }
};

// ─── Implementation ───────────────────────────────────────────────────────────

vtkCameraRotateAxisSource::vtkCameraRotateAxisSource()
{
  this->Center[0] = 0.0; this->Center[1] = 0.0; this->Center[2] = 0.0;
  this->Normal[0] = 0.0; this->Normal[1] = 0.0; this->Normal[2] = 1.0;
  this->Direction[0] = 0.0; this->Direction[1] = 1.0; this->Direction[2] = 0.0;
  this->Scale          = 1.0;
  this->GeneratePolyline = true;
  this->GeneratePolygon  = true;
  this->SetNumberOfInputPorts(0);
}

void vtkCameraRotateAxisSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Center: (" << Center[0] << ", " << Center[1] << ", " << Center[2] << ")\n";
  os << indent << "Normal: (" << Normal[0] << ", " << Normal[1] << ", " << Normal[2] << ")\n";
  os << indent << "Direction: (" << Direction[0] << ", " << Direction[1] << ", " << Direction[2] << ")\n";
  os << indent << "Scale: " << Scale << "\n";
  os << indent << "GeneratePolyline: " << (GeneratePolyline ? "on" : "off") << "\n";
  os << indent << "GeneratePolygon: "  << (GeneratePolygon  ? "on" : "off") << "\n";
}

int vtkCameraRotateAxisSource::RequestData(
  vtkInformation*,
  vtkInformationVector**,
  vtkInformationVector* outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);

  // ── Build a local orthonormal frame ──────────────────────────────────────
  // nrm = normalised Normal
  // ay  = in-plane Y axis (Direction projected onto the normal plane, normalised)
  //        — this is the direction the shape "points to"
  // ax  = ay × nrm  (in-plane X axis, perpendicular to ay)

  double nrm[3] = { Normal[0], Normal[1], Normal[2] };
  double len = vtkMath::Normalize(nrm);
  if (len < 1e-15)
  {
    vtkErrorMacro("Normal vector has zero length.");
    return 0;
  }

  double d[3] = { Direction[0], Direction[1], Direction[2] };
  // project d onto the plane perpendicular to nrm
  double dot = vtkMath::Dot(d, nrm);
  d[0] -= dot * nrm[0];
  d[1] -= dot * nrm[1];
  d[2] -= dot * nrm[2];
  len = vtkMath::Normalize(d);
  if (len < 1e-15)
  {
    vtkErrorMacro("Direction vector is parallel to Normal or has zero length.");
    return 0;
  }
  double ay[3] = { d[0], d[1], d[2] };

  double ax[3];
  vtkMath::Cross(ay, nrm, ax);  // ax = ay × nrm

  // ── Allocate output ───────────────────────────────────────────────────────
  int totalPts = 0;
  for (int ci = 0; ci < ContourCount; ++ci)
    totalPts += ContourPointCounts[ci];

  vtkNew<vtkPoints> pts;
  pts->SetDataTypeToDouble();
  pts->Allocate(totalPts);

  vtkNew<vtkCellArray> polys;
  vtkNew<vtkCellArray> lines;

  // ── Insert points and cells per contour ──────────────────────────────────
  int offset = 0;
  for (int ci = 0; ci < ContourCount; ++ci)
  {
    const int n_pts = ContourPointCounts[ci];
    const vtkIdType startId = pts->GetNumberOfPoints();

    for (int pi = 0; pi < n_pts; ++pi)
    {
      const double u = ContourPoints[offset + pi][0] * Scale;
      const double v = ContourPoints[offset + pi][1] * Scale;
      pts->InsertNextPoint(
        Center[0] + u * ax[0] + v * ay[0],
        Center[1] + u * ax[1] + v * ay[1],
        Center[2] + u * ax[2] + v * ay[2]);
    }

    if (GeneratePolygon)
    {
      polys->InsertNextCell(n_pts);
      for (int pi = 0; pi < n_pts; ++pi)
        polys->InsertCellPoint(startId + pi);
    }

    if (GeneratePolyline)
    {
      // Closed polyline: n_pts vertices + 1 to close back to the first point
      lines->InsertNextCell(n_pts + 1);
      for (int pi = 0; pi < n_pts; ++pi)
        lines->InsertCellPoint(startId + pi);
      lines->InsertCellPoint(startId);  // close
    }

    offset += n_pts;
  }

  // ── Triangulate concave polygons for correct rendering ─────────────────
  // VTK's default OpenGL tessellator can fail on complex concave polygons.
  // We use vtkTriangleFilter (ear-clipping) to produce proper triangles.
  if (GeneratePolygon)
  {
    vtkNew<vtkPolyData> tempPd;
    tempPd->SetPoints(pts);
    tempPd->SetPolys(polys);
    if (GeneratePolyline)
      tempPd->SetLines(lines);

    vtkNew<vtkTriangleFilter> triFilter;
    triFilter->SetInputData(tempPd);
    triFilter->PassVertsOff();
    triFilter->PassLinesOn();
    triFilter->Update();

    output->ShallowCopy(triFilter->GetOutput());
  }
  else
  {
    output->SetPoints(pts);
    if (GeneratePolyline)
      output->SetLines(lines);
  }

  return 1;
}

/**
 * vtkCameraRotateAxisSource.cxx
 *
 * Geometry baked from rotate_axis.svg (viewBox="0 0 323 16").
 * The 2 SVG sub-paths were tessellated into 138 sample points
 * (cubic Bezier curves subdivided into 32 segments each).
 * Coordinates are centred at (0, 0) and normalised by max(width, height) = 323
 * so that the original 323:16 aspect ratio is preserved.
 * The longer axis (X) spans [-0.5, +0.5]; the shorter axis (Y) spans
 * approximately [-0.025, +0.025].
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
  {    -0.46169907,     0.00186372 },
  {    -0.24380402,     0.00292149 },
  {    -0.24380729,     0.00274256 },
  {    -0.24381045,     0.00256359 },
  {    -0.24381351,     0.00238459 },
  {    -0.24381647,     0.00220555 },
  {    -0.24381932,     0.00202648 },
  {    -0.24382207,     0.00184738 },
  {    -0.24382471,     0.00166824 },
  {    -0.24382723,     0.00148907 },
  {    -0.24382965,     0.00130986 },
  {    -0.24383195,     0.00113062 },
  {    -0.24383413,     0.00095135 },
  {    -0.24383619,     0.00077205 },
  {    -0.24383813,     0.00059271 },
  {    -0.24383994,     0.00041335 },
  {    -0.24384163,     0.00023395 },
  {    -0.24384319,     0.00005452 },
  {    -0.24384462,    -0.00012494 },
  {    -0.24384591,    -0.00030443 },
  {    -0.24384708,    -0.00048395 },
  {    -0.24384810,    -0.00066350 },
  {    -0.24384899,    -0.00084307 },
  {    -0.24384973,    -0.00102268 },
  {    -0.24385033,    -0.00120232 },
  {    -0.24385078,    -0.00138198 },
  {    -0.24385109,    -0.00156167 },
  {    -0.24385125,    -0.00174139 },
  {    -0.24385125,    -0.00192113 },
  {    -0.24385110,    -0.00210090 },
  {    -0.24385080,    -0.00228070 },
  {    -0.24385033,    -0.00246052 },
  {    -0.24384970,    -0.00264037 },
  {    -0.24384892,    -0.00282025 },
  {    -0.24384860,    -0.00287973 },
  {    -0.24384825,    -0.00293921 },
  {    -0.24384786,    -0.00299868 },
  {    -0.24384743,    -0.00305815 },
  {    -0.24384696,    -0.00311762 },
  {    -0.24384646,    -0.00317708 },
  {    -0.24384592,    -0.00323654 },
  {    -0.24384536,    -0.00329600 },
  {    -0.24384477,    -0.00335545 },
  {    -0.24384415,    -0.00341490 },
  {    -0.24384350,    -0.00347435 },
  {    -0.24384284,    -0.00353379 },
  {    -0.24384215,    -0.00359323 },
  {    -0.24384144,    -0.00365266 },
  {    -0.24384072,    -0.00371210 },
  {    -0.24383998,    -0.00377152 },
  {    -0.24383922,    -0.00383095 },
  {    -0.24383846,    -0.00389037 },
  {    -0.24383768,    -0.00394979 },
  {    -0.24383690,    -0.00400920 },
  {    -0.24383611,    -0.00406862 },
  {    -0.24383531,    -0.00412802 },
  {    -0.24383452,    -0.00418743 },
  {    -0.24383372,    -0.00424683 },
  {    -0.24383293,    -0.00430623 },
  {    -0.24383213,    -0.00436562 },
  {    -0.24383135,    -0.00442501 },
  {    -0.24383057,    -0.00448440 },
  {    -0.24382980,    -0.00454379 },
  {    -0.24382904,    -0.00460317 },
  {    -0.24382829,    -0.00466254 },
  {    -0.24382755,    -0.00472192 },
  {    -0.46166192,    -0.00577938 },
  {    -0.49978580,    -0.02420495 },
  {    -0.50000000,     0.01991824 },
  // contour 1
  {     0.49889474,    -0.01935697 },
  {     0.46059133,    -0.00130229 },
  {     0.24146749,    -0.00236604 },
  {     0.24146751,    -0.00230666 },
  {     0.24146756,    -0.00224727 },
  {     0.24146764,    -0.00218789 },
  {     0.24146775,    -0.00212850 },
  {     0.24146789,    -0.00206910 },
  {     0.24146804,    -0.00200971 },
  {     0.24146822,    -0.00195031 },
  {     0.24146841,    -0.00189091 },
  {     0.24146862,    -0.00183150 },
  {     0.24146883,    -0.00177209 },
  {     0.24146906,    -0.00171268 },
  {     0.24146929,    -0.00165327 },
  {     0.24146952,    -0.00159385 },
  {     0.24146975,    -0.00153443 },
  {     0.24146998,    -0.00147500 },
  {     0.24147020,    -0.00141557 },
  {     0.24147041,    -0.00135614 },
  {     0.24147061,    -0.00129671 },
  {     0.24147080,    -0.00123727 },
  {     0.24147097,    -0.00117783 },
  {     0.24147112,    -0.00111838 },
  {     0.24147124,    -0.00105894 },
  {     0.24147134,    -0.00099948 },
  {     0.24147141,    -0.00094003 },
  {     0.24147145,    -0.00088057 },
  {     0.24147145,    -0.00082111 },
  {     0.24147142,    -0.00076165 },
  {     0.24147134,    -0.00070218 },
  {     0.24147123,    -0.00064271 },
  {     0.24147106,    -0.00058323 },
  {     0.24147085,    -0.00052375 },
  {     0.24147059,    -0.00046427 },
  {     0.24146965,    -0.00028440 },
  {     0.24146856,    -0.00010455 },
  {     0.24146733,     0.00007527 },
  {     0.24146595,     0.00025506 },
  {     0.24146443,     0.00043483 },
  {     0.24146277,     0.00061457 },
  {     0.24146097,     0.00079427 },
  {     0.24145903,     0.00097396 },
  {     0.24145694,     0.00115361 },
  {     0.24145472,     0.00133323 },
  {     0.24145236,     0.00151282 },
  {     0.24144985,     0.00169239 },
  {     0.24144721,     0.00187192 },
  {     0.24144443,     0.00205142 },
  {     0.24144152,     0.00223089 },
  {     0.24143847,     0.00241033 },
  {     0.24143528,     0.00258974 },
  {     0.24143196,     0.00276911 },
  {     0.24142850,     0.00294845 },
  {     0.24142490,     0.00312776 },
  {     0.24142118,     0.00330704 },
  {     0.24141732,     0.00348628 },
  {     0.24141333,     0.00366549 },
  {     0.24140920,     0.00384466 },
  {     0.24140494,     0.00402380 },
  {     0.24140056,     0.00420291 },
  {     0.24139604,     0.00438198 },
  {     0.24139139,     0.00456101 },
  {     0.24138662,     0.00474001 },
  {     0.24138171,     0.00491897 },
  {     0.24137668,     0.00509789 },
  {     0.24137152,     0.00527678 },
  {     0.46055418,     0.00634080 },
  {     0.49867802,     0.02476635 }
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

/**
 * vtkDSARotateHandleSource.cxx
 *
 * Geometry baked from rotate_handle.svg (viewBox="0 0 21.0 25.0").
 * The 2 SVG paths were tessellated into 177 sample points
 * (cubic Bezier curves subdivided into 32 segments each).
 * Coordinates are normalised to the range [-0.5, +0.5] and centred at (0, 0).
 * The SVG Y-axis (pointing down) is flipped so that the local +Y axis points up.
 */

#include "vtkDSARotateHandleSource.h"

#include <vtkCellArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

vtkStandardNewMacro(vtkDSARotateHandleSource);

// ─── Baked geometry ──────────────────────────────────────────────────────────

static const int ContourCount = 2;

static const int ContourPointCounts[ContourCount] = {
  98, 79
};

// Total: 177 points across 2 contours.
// Layout: first 98 points are contour 0; next 79 points are contour 1.
static const double ContourPoints[][2] = {
  // contour 0
  { 0.30289200, -0.48648400 },
  { 0.29599753, -0.48649698 },
  { 0.28639842, -0.48653366 },
  { 0.27427929, -0.48659070 },
  { 0.25982474, -0.48666473 },
  { 0.24321937, -0.48675239 },
  { 0.22464778, -0.48685032 },
  { 0.20429457, -0.48695516 },
  { 0.18234435, -0.48706356 },
  { 0.15898172, -0.48717215 },
  { 0.13439129, -0.48727758 },
  { 0.10875765, -0.48737648 },
  { 0.08226541, -0.48746549 },
  { 0.05509917, -0.48754126 },
  { 0.02744354, -0.48760042 },
  { -0.00051689, -0.48763962 },
  { -0.02859750, -0.48765550 },
  { -0.05661370, -0.48764469 },
  { -0.08438089, -0.48760384 },
  { -0.11171445, -0.48752959 },
  { -0.13842980, -0.48741857 },
  { -0.16434232, -0.48726743 },
  { -0.18926741, -0.48707281 },
  { -0.21302047, -0.48683135 },
  { -0.23541690, -0.48653969 },
  { -0.25627209, -0.48619446 },
  { -0.27540145, -0.48579232 },
  { -0.29262036, -0.48532990 },
  { -0.30774423, -0.48480384 },
  { -0.32058845, -0.48421077 },
  { -0.33096842, -0.48354735 },
  { -0.33869954, -0.48281022 },
  { -0.34359720, -0.48199600 },
  { -0.35096736, -0.48001814 },
  { -0.35784889, -0.47789897 },
  { -0.36425847, -0.47565189 },
  { -0.37021281, -0.47329027 },
  { -0.37572861, -0.47082750 },
  { -0.38082258, -0.46827696 },
  { -0.38551141, -0.46565204 },
  { -0.38981180, -0.46296612 },
  { -0.39374046, -0.46023260 },
  { -0.39731409, -0.45746484 },
  { -0.40054938, -0.45467624 },
  { -0.40346304, -0.45188017 },
  { -0.40607177, -0.44909003 },
  { -0.40839227, -0.44631920 },
  { -0.41044124, -0.44358106 },
  { -0.41223538, -0.44088900 },
  { -0.41379140, -0.43825640 },
  { -0.41512599, -0.43569664 },
  { -0.41625586, -0.43322312 },
  { -0.41719770, -0.43084920 },
  { -0.41796822, -0.42858829 },
  { -0.41858412, -0.42645376 },
  { -0.41906210, -0.42445899 },
  { -0.41941886, -0.42261738 },
  { -0.41967110, -0.42094230 },
  { -0.41983552, -0.41944713 },
  { -0.41992883, -0.41814528 },
  { -0.41996772, -0.41705011 },
  { -0.41996889, -0.41617501 },
  { -0.41994906, -0.41553337 },
  { -0.41992491, -0.41513857 },
  { -0.41991314, -0.41500400 },
  { 0.38792800, -0.41500400 },
  { 0.38782593, -0.41520905 },
  { 0.38752219, -0.41580675 },
  { 0.38702045, -0.41677093 },
  { 0.38632440, -0.41807541 },
  { 0.38543771, -0.41969400 },
  { 0.38436408, -0.42160054 },
  { 0.38310718, -0.42376885 },
  { 0.38167069, -0.42617275 },
  { 0.38005829, -0.42878606 },
  { 0.37827368, -0.43158261 },
  { 0.37632052, -0.43453622 },
  { 0.37420251, -0.43762072 },
  { 0.37192332, -0.44080992 },
  { 0.36948663, -0.44407765 },
  { 0.36689613, -0.44739774 },
  { 0.36415550, -0.45074400 },
  { 0.36126842, -0.45409026 },
  { 0.35823857, -0.45741035 },
  { 0.35506964, -0.46067808 },
  { 0.35176530, -0.46386728 },
  { 0.34832925, -0.46695178 },
  { 0.34476515, -0.46990539 },
  { 0.34107669, -0.47270194 },
  { 0.33726756, -0.47531525 },
  { 0.33334144, -0.47771915 },
  { 0.32930200, -0.47988746 },
  { 0.32515293, -0.48179400 },
  { 0.32089791, -0.48341259 },
  { 0.31654063, -0.48471707 },
  { 0.31208476, -0.48568125 },
  { 0.30753399, -0.48627895 },
  { 0.30289200, -0.48648400 },
  // contour 1
  { 0.28480400, 0.47070956 },
  { 0.25734400, 0.49999994 },
  { 0.02668400, 0.49999996 },
  { -0.27537400, 0.49999999 },
  { -0.27546037, 0.49995580 },
  { -0.27571366, 0.49982523 },
  { -0.27612509, 0.49961132 },
  { -0.27668592, 0.49931706 },
  { -0.27738737, 0.49894549 },
  { -0.27822070, 0.49849961 },
  { -0.27917714, 0.49798245 },
  { -0.28024794, 0.49739701 },
  { -0.28142433, 0.49674633 },
  { -0.28269755, 0.49603341 },
  { -0.28405885, 0.49526126 },
  { -0.28549946, 0.49443292 },
  { -0.28701064, 0.49355139 },
  { -0.28858361, 0.49261969 },
  { -0.29020961, 0.49164085 },
  { -0.29187990, 0.49061786 },
  { -0.29358571, 0.48955376 },
  { -0.29531827, 0.48845156 },
  { -0.29706884, 0.48731428 },
  { -0.29882865, 0.48614493 },
  { -0.30058894, 0.48494652 },
  { -0.30234095, 0.48372209 },
  { -0.30407593, 0.48247464 },
  { -0.30578511, 0.48120719 },
  { -0.30745974, 0.47992276 },
  { -0.30909105, 0.47862436 },
  { -0.31067029, 0.47731501 },
  { -0.31218870, 0.47599773 },
  { -0.31363751, 0.47467554 },
  { -0.31500797, 0.47335145 },
  { -0.31629132, 0.47202848 },
  { -0.31747880, 0.47070964 },
  { -0.31860851, 0.46935404 },
  { -0.31972348, 0.46792391 },
  { -0.32082232, 0.46642697 },
  { -0.32190363, 0.46487091 },
  { -0.32296605, 0.46326346 },
  { -0.32400818, 0.46161232 },
  { -0.32502864, 0.45992520 },
  { -0.32602605, 0.45820981 },
  { -0.32699901, 0.45647386 },
  { -0.32794615, 0.45472506 },
  { -0.32886608, 0.45297112 },
  { -0.32975742, 0.45121975 },
  { -0.33061877, 0.44947866 },
  { -0.33144876, 0.44775556 },
  { -0.33224600, 0.44605815 },
  { -0.33300910, 0.44439416 },
  { -0.33373668, 0.44277128 },
  { -0.33442736, 0.44119722 },
  { -0.33507976, 0.43967971 },
  { -0.33569247, 0.43822644 },
  { -0.33626413, 0.43684513 },
  { -0.33679334, 0.43554349 },
  { -0.33727873, 0.43432922 },
  { -0.33771890, 0.43321003 },
  { -0.33811247, 0.43219364 },
  { -0.33845806, 0.43128776 },
  { -0.33875428, 0.43050009 },
  { -0.33899975, 0.42983834 },
  { -0.33919308, 0.42931023 },
  { -0.33933289, 0.42892347 },
  { -0.33941779, 0.42868575 },
  { -0.33944640, 0.42860480 },
  { -0.33944640, 0.24462480 },
  { -0.33944640, -0.01075200 },
  { -0.39436596, -0.06933200 },
  { -0.41999502, -0.14988000 },
  { -0.41999504, -0.37504800 },
  { 0.39281200, -0.37504800 },
  { 0.39281200, -0.13340400 },
  { 0.35620000, -0.06200800 },
  { 0.30128000, -0.01075200 },
  { 0.30128000, 0.43592720 },
  { 0.28480400, 0.47070956 },
};

// ─── Implementation ───────────────────────────────────────────────────────────

vtkDSARotateHandleSource::vtkDSARotateHandleSource()
{
  this->Center[0] = 0.0; this->Center[1] = 0.0; this->Center[2] = 0.0;
  this->Normal[0] = 0.0; this->Normal[1] = 0.0; this->Normal[2] = 1.0;
  this->Direction[0] = 1.0; this->Direction[1] = 0.0; this->Direction[2] = 0.0;
  this->Scale          = 1.0;
  this->GeneratePolyline = true;
  this->GeneratePolygon  = true;
  this->SetNumberOfInputPorts(0);
}

void vtkDSARotateHandleSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Center: (" << Center[0] << ", " << Center[1] << ", " << Center[2] << ")\n";
  os << indent << "Normal: (" << Normal[0] << ", " << Normal[1] << ", " << Normal[2] << ")\n";
  os << indent << "Direction: (" << Direction[0] << ", " << Direction[1] << ", " << Direction[2] << ")\n";
  os << indent << "Scale: " << Scale << "\n";
  os << indent << "GeneratePolyline: " << (GeneratePolyline ? "on" : "off") << "\n";
  os << indent << "GeneratePolygon: "  << (GeneratePolygon  ? "on" : "off") << "\n";
}

int vtkDSARotateHandleSource::RequestData(
  vtkInformation*,
  vtkInformationVector**,
  vtkInformationVector* outputVector)
{
  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);

  // ── Build a local orthonormal frame ──────────────────────────────────────
  // nrm = normalised Normal
  // ax  = in-plane X axis (Direction projected onto the normal plane, normalised)
  // ay  = nrm × ax  (in-plane Y axis)

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
  double ax[3] = { d[0], d[1], d[2] };

  double ay[3];
  vtkMath::Cross(nrm, ax, ay);  // ay = nrm × ax

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

  output->SetPoints(pts);
  if (GeneratePolygon)
    output->SetPolys(polys);
  if (GeneratePolyline)
    output->SetLines(lines);

  return 1;
}

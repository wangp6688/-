#ifndef vtkTrajectoryRoundCapSource_h
#define vtkTrajectoryRoundCapSource_h

#include <array>
#include <vector>

#include <vtkPolyDataAlgorithm.h>
class vtkPoints;

/**
 * @class vtkTrajectoryRoundCapSource
 * @brief Build 2D trajectory polydata with round caps in a plane.
 *
 * The source consumes an internal trajectory point list and generates a filled
 * stroke polygon in the plane defined by Normal. The stroke width is 2*Radius
 * and both ends are semicircle caps.
 */
class vtkTrajectoryRoundCapSource : public vtkPolyDataAlgorithm
{
public:
  static vtkTrajectoryRoundCapSource* New();
  vtkTypeMacro(vtkTrajectoryRoundCapSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);
  ///@}

  ///@{
  vtkSetClampMacro(Radius, double, 1e-9, VTK_DOUBLE_MAX);
  vtkGetMacro(Radius, double);
  ///@}

  ///@{
  vtkSetClampMacro(CapResolution, int, 3, 360);
  vtkGetMacro(CapResolution, int);
  ///@}

  void ClearPoints();
  void AddPoint(double x, double y, double z);
  void SetPoints(vtkPoints* points);
  vtkIdType GetNumberOfTrajectoryPoints() const;

protected:
  vtkTrajectoryRoundCapSource();
  ~vtkTrajectoryRoundCapSource() override = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkTrajectoryRoundCapSource(const vtkTrajectoryRoundCapSource&) = delete;
  void operator=(const vtkTrajectoryRoundCapSource&) = delete;

  std::vector<std::array<double, 3>> TrajectoryPoints;
  double Normal[3];
  double Radius;
  int CapResolution;
};

#endif // vtkTrajectoryRoundCapSource_h

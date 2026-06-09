#ifndef vtkEyeBallSource_h
#define vtkEyeBallSource_h

#include <vtkPolyDataAlgorithm.h>

/**
 * @class vtkEyeBallSource
 * @brief Generates the eye-ball SVG shape as vtkPolyData.
 *
 * Geometry is baked from EyeBall.svg at code-generation time.
 * No runtime SVG parsing is performed.
 */
class vtkEyeBallSource : public vtkPolyDataAlgorithm
{
public:
  static vtkEyeBallSource* New();
  vtkTypeMacro(vtkEyeBallSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkSetVector3Macro(Center, double);
  vtkGetVector3Macro(Center, double);

  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);

  vtkSetVector3Macro(Direction, double);
  vtkGetVector3Macro(Direction, double);

  vtkSetClampMacro(Scale, double, 1e-9, VTK_DOUBLE_MAX);
  vtkGetMacro(Scale, double);

  vtkSetMacro(GeneratePolyline, bool);
  vtkGetMacro(GeneratePolyline, bool);
  vtkBooleanMacro(GeneratePolyline, bool);

  vtkSetMacro(GeneratePolygon, bool);
  vtkGetMacro(GeneratePolygon, bool);
  vtkBooleanMacro(GeneratePolygon, bool);

protected:
  vtkEyeBallSource();
  ~vtkEyeBallSource() override = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkEyeBallSource(const vtkEyeBallSource&) = delete;
  void operator=(const vtkEyeBallSource&) = delete;

  double Center[3];
  double Normal[3];
  double Direction[3];
  double Scale;
  bool GeneratePolyline;
  bool GeneratePolygon;
};

#endif // vtkEyeBallSource_h

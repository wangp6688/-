#ifndef vtkEyePupilSource_h
#define vtkEyePupilSource_h

#include <vtkPolyDataAlgorithm.h>

/**
 * @class vtkEyePupilSource
 * @brief Generates the eye pupil SVG shape as a vtkPolyData.
 *
 * The shape geometry is derived from EyePupil.svg and baked in as static
 * arrays at compile time — no SVG parsing occurs at runtime.
 *
 * Port 0 always outputs the filled polygon. When GeneratePolyline is enabled,
 * port 1 additionally outputs the closed contour polyline.
 *
 * GeneratePolyline defaults to off so the source keeps its historical single
 * output unless the caller explicitly opts in.
 */
class vtkEyePupilSource : public vtkPolyDataAlgorithm
{
public:
  static vtkEyePupilSource* New();
  vtkTypeMacro(vtkEyePupilSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  vtkSetVector3Macro(Center, double);
  vtkGetVector3Macro(Center, double);
  ///@}

  ///@{
  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);
  ///@}

  ///@{
  vtkSetVector3Macro(Direction, double);
  vtkGetVector3Macro(Direction, double);
  ///@}

  ///@{
  vtkSetClampMacro(Scale, double, 1e-9, VTK_DOUBLE_MAX);
  vtkGetMacro(Scale, double);
  ///@}

  ///@{
  void SetGeneratePolyline(bool generatePolyline);
  vtkGetMacro(GeneratePolyline, bool);
  vtkBooleanMacro(GeneratePolyline, bool);
  ///@}

  ///@{
  vtkSetMacro(GeneratePolygon, bool);
  vtkGetMacro(GeneratePolygon, bool);
  vtkBooleanMacro(GeneratePolygon, bool);
  ///@}

protected:
  vtkEyePupilSource();
  ~vtkEyePupilSource() override = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  int FillOutputPortInformation(int port, vtkInformation* info) override;

private:
  vtkEyePupilSource(const vtkEyePupilSource&) = delete;
  void operator=(const vtkEyePupilSource&) = delete;

  double Center[3];
  double Normal[3];
  double Direction[3];
  double Scale;
  bool GeneratePolyline;
  bool GeneratePolygon;
};

#endif // vtkEyePupilSource_h

#ifndef vtkCameraRotateAxisSource_h
#define vtkCameraRotateAxisSource_h

#include <vtkPolyDataAlgorithm.h>

/**
 * @class vtkCameraRotateAxisSource
 * @brief Generates the rotate-axis SVG shape as a vtkPolyData.
 *
 * The shape geometry is derived from rotate_axis.svg and baked in as static
 * arrays at compile time — no SVG parsing occurs at runtime.
 *
 * The 2-D outline is placed in 3-D space using three user-settable parameters:
 *   - Center    – origin of the shape in world space.
 *   - Normal    – normal of the plane in which the shape lies.
 *   - Direction – in-plane direction that the shape "points to" (maps to
 *                 the local +Y axis of the SVG, i.e. the shape's primary /
 *                 tall axis; automatically projected onto the Normal plane).
 *
 * An optional Scale factor uniformly scales the shape around Center.
 *
 * Two output modes are independently controllable:
 *   - GeneratePolyline (default: on)  – closed outline polylines.
 *   - GeneratePolygon  (default: on)  – filled polygon cells.
 *
 * Typical usage:
 * @code
 *   auto src = vtkSmartPointer<vtkCameraRotateAxisSource>::New();
 *   src->SetCenter(0.0, 0.0, 0.0);
 *   src->SetNormal(0.0, 0.0, 1.0);
 *   src->SetDirection(0.0, 1.0, 0.0);
 *   src->SetScale(50.0);
 *   src->Update();
 *   vtkPolyData* pd = src->GetOutput();
 * @endcode
 */
class vtkCameraRotateAxisSource : public vtkPolyDataAlgorithm
{
public:
  static vtkCameraRotateAxisSource* New();
  vtkTypeMacro(vtkCameraRotateAxisSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /** World-space center of the shape. Default: (0, 0, 0). */
  vtkSetVector3Macro(Center, double);
  vtkGetVector3Macro(Center, double);
  ///@}

  ///@{
  /** Normal of the plane in which the shape lies. Default: (0, 0, 1). */
  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);
  ///@}

  ///@{
  /** In-plane direction that the shape "points to" (mapped to the local +Y
   *  axis of the SVG, i.e. the shape's primary / tall axis).
   *  It is automatically projected onto the Normal plane.
   *  Default: (0, 1, 0). */
  vtkSetVector3Macro(Direction, double);
  vtkGetVector3Macro(Direction, double);
  ///@}

  ///@{
  /** Uniform scale applied to all shape coordinates. Default: 1.0. */
  vtkSetClampMacro(Scale, double, 1e-9, VTK_DOUBLE_MAX);
  vtkGetMacro(Scale, double);
  ///@}

  ///@{
  /** Enable / disable closed polyline output (outline). Default: on. */
  vtkSetMacro(GeneratePolyline, bool);
  vtkGetMacro(GeneratePolyline, bool);
  vtkBooleanMacro(GeneratePolyline, bool);
  ///@}

  ///@{
  /** Enable / disable filled polygon output. Default: on. */
  vtkSetMacro(GeneratePolygon, bool);
  vtkGetMacro(GeneratePolygon, bool);
  vtkBooleanMacro(GeneratePolygon, bool);
  ///@}

protected:
  vtkCameraRotateAxisSource();
  ~vtkCameraRotateAxisSource() override = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkCameraRotateAxisSource(const vtkCameraRotateAxisSource&) = delete;
  void operator=(const vtkCameraRotateAxisSource&) = delete;

  double Center[3];
  double Normal[3];
  double Direction[3];
  double Scale;
  bool   GeneratePolyline;
  bool   GeneratePolygon;
};

#endif // vtkCameraRotateAxisSource_h

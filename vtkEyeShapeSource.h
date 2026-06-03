#ifndef vtkEyeShapeSource_h
#define vtkEyeShapeSource_h

#include <vtkPolyDataAlgorithm.h>

/**
 * @class vtkEyeShapeSource
 * @brief Generates the eye SVG shape as a vtkPolyData.
 *
 * The shape geometry is derived from eye.svg (viewBox="0 0 25 19") and baked
 * in as static arrays at compile time — no SVG parsing occurs at runtime.
 *
 * The SVG contains two closed sub-paths:
 *   - Contour 0: outer eye outline (almond/lens silhouette).
 *   - Contour 1: inner pupil circle.
 *
 * Both sub-paths are tessellated into dense polylines (cubic Bezier curves
 * subdivided into 32 segments each) and then normalised to the range
 * [-0.5, +0.5] around the viewBox centre (12.5, 9.5).  The SVG Y-axis
 * (pointing down) is flipped so that the local +Y axis points up.
 *
 * The 2-D outline is placed in 3-D space using three user-settable parameters:
 *   - Center    – origin of the shape in world space.
 *   - Normal    – normal of the plane in which the shape lies.
 *   - Direction – in-plane direction that the shape "points to" (maps to
 *                 the local +Y axis of the SVG; automatically projected onto
 *                 the Normal plane).
 *
 * An optional Scale factor uniformly scales the shape around Center.
 *
 * Two output modes are independently controllable:
 *   - GeneratePolyline (default: on)  – closed outline polylines.
 *   - GeneratePolygon  (default: on)  – filled polygon cells.
 *
 * Typical usage:
 * @code
 *   auto src = vtkSmartPointer<vtkEyeShapeSource>::New();
 *   src->SetCenter(0.0, 0.0, 0.0);
 *   src->SetNormal(0.0, 0.0, 1.0);
 *   src->SetDirection(0.0, 1.0, 0.0);
 *   src->SetScale(50.0);
 *   src->Update();
 *   vtkPolyData* pd = src->GetOutput();
 * @endcode
 */
class vtkEyeShapeSource : public vtkPolyDataAlgorithm
{
public:
  static vtkEyeShapeSource* New();
  vtkTypeMacro(vtkEyeShapeSource, vtkPolyDataAlgorithm);
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
  vtkEyeShapeSource();
  ~vtkEyeShapeSource() override = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkEyeShapeSource(const vtkEyeShapeSource&) = delete;
  void operator=(const vtkEyeShapeSource&) = delete;

  double Center[3];
  double Normal[3];
  double Direction[3];
  double Scale;
  bool   GeneratePolyline;
  bool   GeneratePolygon;
};

#endif // vtkEyeShapeSource_h

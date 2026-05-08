#ifndef vtkOutlinedTextActor_h
#define vtkOutlinedTextActor_h

#include <array>

#include <vtkSmartPointer.h>
#include <vtkTextActor.h>

class vtkViewport;
class vtkWindow;

class vtkOutlinedTextActor : public vtkTextActor
{
public:
  static vtkOutlinedTextActor* New();
  vtkTypeMacro(vtkOutlinedTextActor, vtkTextActor);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /** Color used by the outline text actors. */
  vtkSetVector3Macro(OutlineColor, double);
  vtkGetVector3Macro(OutlineColor, double);
  ///@}

  ///@{
  /** Opacity of the outline text actors. */
  vtkSetClampMacro(OutlineOpacity, double, 0.0, 1.0);
  vtkGetMacro(OutlineOpacity, double);
  ///@}

  ///@{
  /** Pixel offset used for each outline actor. */
  vtkSetClampMacro(OutlineOffset, int, 0, VTK_INT_MAX);
  vtkGetMacro(OutlineOffset, int);
  ///@}

  ///@{
  /** Toggle outline visibility. */
  vtkSetMacro(OutlineVisibility, bool);
  vtkGetMacro(OutlineVisibility, bool);
  vtkBooleanMacro(OutlineVisibility, bool);
  ///@}

  int RenderOverlay(vtkViewport* viewport) override;
  int RenderOpaqueGeometry(vtkViewport* viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport* viewport) override;
  vtkTypeBool HasTranslucentPolygonalGeometry() override;
  void ReleaseGraphicsResources(vtkWindow* window) override;

protected:
  vtkOutlinedTextActor();
  ~vtkOutlinedTextActor() override = default;

  void SyncOutlineText();
  void SyncOutlinePositions(vtkViewport* viewport);
  void SyncOutlineActors(vtkViewport* viewport);

private:
  vtkOutlinedTextActor(const vtkOutlinedTextActor&) = delete;
  void operator=(const vtkOutlinedTextActor&) = delete;

  std::array<vtkSmartPointer<vtkTextActor>, 8> BorderActors;
  double OutlineColor[3];
  double OutlineOpacity;
  int OutlineOffset;
  bool OutlineVisibility;
};

#endif // vtkOutlinedTextActor_h

/**
 * Example_RotateHandle.cxx
 *
 * Demonstrates vtkDSARotateHandleSource.
 *
 * The source generates the rotate-handle shape (from rotate_handle.svg) as a
 * vtkPolyData placed in 3-D space.  Center, Normal, Direction, and Scale are
 * all settable; the geometry itself is baked in at compile time with no
 * runtime SVG parsing.
 *
 * Build with the provided CMakeLists.txt, then run:
 *   ./Example_RotateHandle
 * Add --no-render to skip the interactive window (useful in CI).
 */

#include "vtkDSARotateHandleSource.h"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
  bool noRender = false;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--no-render")
      noRender = true;
  }

  // ── 1. Create the shape source ────────────────────────────────────────────
  vtkNew<vtkDSARotateHandleSource> src;
  src->SetCenter(0.0, 0.0, 0.0);    // centred at origin
  src->SetNormal(0.0, 0.0, 1.0);    // shape lies in the XY plane
  src->SetDirection(1.0, 0.0, 0.0); // SVG +X maps to world +X
  src->SetScale(50.0);               // scale to ~50 units wide
  src->GeneratePolygonOn();
  src->GeneratePolylineOn();
  src->Update();

  vtkPolyData* pd = src->GetOutput();
  std::cout << "vtkDSARotateHandleSource output:\n";
  std::cout << "  Points : " << pd->GetNumberOfPoints() << "\n";
  std::cout << "  Cells  : " << pd->GetNumberOfCells()  << "\n";

  if (noRender)
  {
    std::cout << "(Rendering skipped via --no-render)\n";
    return EXIT_SUCCESS;
  }

  // ── 2. Visualise ──────────────────────────────────────────────────────────
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(pd);
  mapper->ScalarVisibilityOff();

  // Filled polygon actor (white, translucent)
  vtkNew<vtkActor> polyActor;
  polyActor->SetMapper(mapper);
  polyActor->GetProperty()->SetColor(0.9, 0.9, 0.9);
  polyActor->GetProperty()->SetOpacity(0.7);

  // Outline actor (blue, solid line on top)
  vtkNew<vtkPolyDataMapper> lineMapper;
  lineMapper->SetInputData(pd);
  lineMapper->ScalarVisibilityOff();

  vtkNew<vtkActor> lineActor;
  lineActor->SetMapper(lineMapper);
  lineActor->GetProperty()->SetRepresentationToWireframe();
  lineActor->GetProperty()->SetColor(0.2, 0.5, 1.0);
  lineActor->GetProperty()->SetLineWidth(2.0);

  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(0.15, 0.15, 0.15);
  renderer->AddActor(polyActor);
  renderer->AddActor(lineActor);
  renderer->ResetCamera();

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 800);
  renderWindow->SetWindowName("vtkDSARotateHandleSource");

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  interactor->SetInteractorStyle(style);

  renderWindow->Render();
  std::cout << "\nClose the render window to exit.\n";
  interactor->Start();

  return EXIT_SUCCESS;
}

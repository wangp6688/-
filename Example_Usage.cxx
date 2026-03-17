/**
 * Example_Usage.cxx
 *
 * Demonstrates vtkImageLabelContourExtractor on a 512x512 synthetic image
 * containing ~200 circular label regions.
 *
 * Build with the provided CMakeLists.txt, then run:
 *   ./Example
 */

#include "vtkImageLabelContourExtractor.h"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkDoubleArray.h>
#include <vtkFieldData.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSMPTools.h>
#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

// ──────────────────────────────────────────────────────────────────────────────
//  Helper: Convert HSV → RGB (h in [0,1], s and v in [0,1])
// ──────────────────────────────────────────────────────────────────────────────
static void HSVtoRGB(double h, double s, double v, double rgb[3])
{
  if (s <= 0.0)
  {
    rgb[0] = rgb[1] = rgb[2] = v;
    return;
  }
  double hh = h * 6.0;
  if (hh >= 6.0) hh = 0.0;
  int i = static_cast<int>(hh);
  double ff = hh - i;
  double p = v * (1.0 - s);
  double q = v * (1.0 - s * ff);
  double t = v * (1.0 - s * (1.0 - ff));
  switch (i)
  {
    case 0: rgb[0] = v; rgb[1] = t; rgb[2] = p; break;
    case 1: rgb[0] = q; rgb[1] = v; rgb[2] = p; break;
    case 2: rgb[0] = p; rgb[1] = v; rgb[2] = t; break;
    case 3: rgb[0] = p; rgb[1] = q; rgb[2] = v; break;
    case 4: rgb[0] = t; rgb[1] = p; rgb[2] = v; break;
    default: rgb[0] = v; rgb[1] = p; rgb[2] = q; break;
  }
}

// ──────────────────────────────────────────────────────────────────────────────
//  Create a 512×512 test label image with ~200 circular regions
// ──────────────────────────────────────────────────────────────────────────────
static vtkSmartPointer<vtkImageData> CreateTestLabelImage()
{
  const int W = 512;
  const int H = 512;
  const int numCircles = 200;

  auto image = vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(W, H, 1);
  image->SetSpacing(1.0, 1.0, 1.0);
  image->SetOrigin(0.0, 0.0, 0.0);
  image->AllocateScalars(VTK_SHORT, 1);

  // Initialise all pixels to 0 (background).
  short* ptr = static_cast<short*>(image->GetScalarPointer());
  std::fill(ptr, ptr + W * H, static_cast<short>(0));

  // Seed deterministic positions and radii for reproducibility.
  // Use simple LCG to avoid <random> dependency issues on older compilers.
  unsigned int rng = 42u;
  auto lcg = [&]() -> unsigned int {
    rng = rng * 1664525u + 1013904223u;
    return rng;
  };

  for (int c = 0; c < numCircles; ++c)
  {
    short label = static_cast<short>(c + 1);               // labels 1..200
    double cx = (lcg() % (W - 60)) + 30.0;
    double cy = (lcg() % (H - 60)) + 30.0;
    double r  = (lcg() % 20) + 8.0;                        // radius 8–27 px

    int x0 = std::max(0, static_cast<int>(cx - r - 1));
    int x1 = std::min(W - 1, static_cast<int>(cx + r + 1));
    int y0 = std::max(0, static_cast<int>(cy - r - 1));
    int y1 = std::min(H - 1, static_cast<int>(cy + r + 1));

    for (int y = y0; y <= y1; ++y)
    {
      for (int x = x0; x <= x1; ++x)
      {
        double dx = x - cx;
        double dy = y - cy;
        if (dx * dx + dy * dy <= r * r)
        {
          ptr[y * W + x] = label;
        }
      }
    }
  }

  return image;
}

// ──────────────────────────────────────────────────────────────────────────────
//  main
// ──────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
  // Optional flag: --no-render to skip the interactive window (useful in CI).
  bool noRender = false;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--no-render")
    {
      noRender = true;
    }
  }

  // ── 1. Create test image ─────────────────────────────────────────────────
  std::cout << "Creating 512x512 test label image with ~200 circular regions...\n";
  vtkSmartPointer<vtkImageData> testImage = CreateTestLabelImage();

  // ── 2. Print SMP backend info ────────────────────────────────────────────
  std::cout << "SMP backend : " << vtkSMPTools::GetBackend() << "\n";
  std::cout << "Thread count: " << vtkSMPTools::GetEstimatedNumberOfThreads() << "\n";

  // ── 3. Run extractor ─────────────────────────────────────────────────────
  vtkNew<vtkImageLabelContourExtractor> extractor;
  extractor->SetInputData(testImage);
  extractor->SetBackgroundValue(0.0);
  extractor->UseBackgroundValueOn();
  extractor->SmoothContoursOff();   // disable smoothing for maximum speed

  vtkNew<vtkTimerLog> timer;
  timer->StartTimer();
  extractor->Update();
  timer->StopTimer();

  const double elapsed = timer->GetElapsedTime();

  // ── 4. Print statistics ───────────────────────────────────────────────────
  vtkMultiBlockDataSet* output = extractor->GetOutput();
  const unsigned int numBlocks = output->GetNumberOfBlocks();

  vtkIdType totalPoints = 0;
  vtkIdType totalCells  = 0;

  for (unsigned int b = 0; b < numBlocks; ++b)
  {
    vtkPolyData* poly = vtkPolyData::SafeDownCast(output->GetBlock(b));
    if (poly)
    {
      totalPoints += poly->GetNumberOfPoints();
      totalCells  += poly->GetNumberOfCells();
    }
  }

  std::cout << "\n=== Results ===\n";
  std::cout << "Extraction time : " << elapsed << " s\n";
  std::cout << "Blocks (labels) : " << numBlocks << "\n";
  std::cout << "Total points    : " << totalPoints << "\n";
  std::cout << "Total cells     : " << totalCells << "\n";

  if (noRender)
  {
    std::cout << "(Rendering skipped via --no-render)\n";
    return EXIT_SUCCESS;
  }

  // ── 5. Visualise ─────────────────────────────────────────────────────────
  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(0.1, 0.1, 0.1);

  // Golden-ratio HSV distribution for visually distinct colours.
  const double goldenRatio = 0.618033988749895;
  double hue = 0.0;

  for (unsigned int b = 0; b < numBlocks; ++b)
  {
    vtkPolyData* poly = vtkPolyData::SafeDownCast(output->GetBlock(b));
    if (!poly || poly->GetNumberOfCells() == 0)
    {
      hue = std::fmod(hue + goldenRatio, 1.0);
      continue;
    }

    vtkDoubleArray* labelArr =
      vtkDoubleArray::SafeDownCast(poly->GetFieldData()->GetArray("LabelValue"));
    double labelVal = labelArr ? labelArr->GetValue(0) : static_cast<double>(b);

    double rgb[3];
    HSVtoRGB(hue, 0.85, 0.95, rgb);
    hue = std::fmod(hue + goldenRatio, 1.0);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->ScalarVisibilityOff();

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(rgb);
    actor->GetProperty()->SetLineWidth(1.5);

    renderer->AddActor(actor);
    (void)labelVal; // suppress unused-variable warning in release builds
  }

  renderer->ResetCamera();

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(1024, 1024);
  renderWindow->SetWindowName("vtkImageLabelContourExtractor – label contours");

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  interactor->SetInteractorStyle(style);

  renderWindow->Render();

  std::cout << "\nClose the render window to exit.\n";
  interactor->Start();

  return EXIT_SUCCESS;
}

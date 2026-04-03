#ifndef vtkImageLabelContourExtractor_h
#define vtkImageLabelContourExtractor_h

#include <vtkMultiBlockDataSetAlgorithm.h>
#include <vtkSmartPointer.h>

#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <vector>

class vtkImageData;
class vtkMultiBlockDataSet;
class vtkPolyData;

/**
 * @class vtkImageLabelContourExtractor
 * @brief High-performance multi-threaded contour extraction for 2D label images.
 *
 * This filter accepts a 2D vtkImageData containing discrete label (pixel) values
 * (e.g., produced by vtkImageReslice from a 3D segmentation volume) and extracts
 * iso-contours for every unique label.
 *
 * ## Two output ports
 *
 * - **Port 0 (Contours)**: A vtkMultiBlockDataSet where each block is a vtkPolyData
 *   containing the contour polyline(s) of one label. Always generated.
 * - **Port 1 (Filled Polygons)**: A vtkMultiBlockDataSet where each block is a
 *   vtkPolyData containing the filled triangulated polygon(s) of one label.
 *   Only generated when GenerateFilledPolygons is true.
 *
 * Retrieve outputs via:
 * @code
 *   extractor->GetOutput(0);  // contours
 *   extractor->GetOutput(1);  // filled polygons
 * @endcode
 *
 * ## 4-phase parallel pipeline (vtkSMPTools)
 *
 * - **Phase 1** – Parallel label collection using vtkSMPThreadLocal.
 * - **Phase 2** – Single-pass O(N) parallel binary-mask generation.
 * - **Phase 3** – Per-label parallel contour extraction (independent VTK pipelines).
 * - **Phase 3b** – Per-label parallel filled polygon generation (if enabled).
 * - **Phase 4** – Serial assembly into vtkMultiBlockDataSet(s).
 *
 * Each output vtkPolyData carries a vtkDoubleArray named "LabelValue" in its
 * FieldData storing the actual pixel value, and the corresponding block is
 * named "Label_<intValue>".
 *
 * ### Typical usage
 * @code
 *   auto extractor = vtkSmartPointer<vtkImageLabelContourExtractor>::New();
 *   extractor->SetInputData(sliceImage);   // 2D vtkImageData
 *   extractor->SetBackgroundValue(0.0);    // skip background
 *   extractor->SmoothContoursOn();         // optional Gaussian smoothing
 *   extractor->GenerateFilledPolygonsOn(); // enable filled polygon output
 *   extractor->Update();
 *   vtkMultiBlockDataSet* contours = extractor->GetOutput(0);
 *   vtkMultiBlockDataSet* filled   = extractor->GetOutput(1);
 * @endcode
 *
 * @note Requires VTK 9.1+. The input must be a 2D image (one extent dimension == 1).
 */
class vtkImageLabelContourExtractor : public vtkMultiBlockDataSetAlgorithm
{
public:
  static vtkImageLabelContourExtractor* New();
  vtkTypeMacro(vtkImageLabelContourExtractor, vtkMultiBlockDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Label value to skip (background).  Default: 0.
   * Only effective when UseBackgroundValue is true.
   */
  vtkSetMacro(BackgroundValue, double);
  vtkGetMacro(BackgroundValue, double);
  ///@}

  ///@{
  /**
   * Enable / disable background-value skipping.  Default: true.
   */
  vtkSetMacro(UseBackgroundValue, bool);
  vtkGetMacro(UseBackgroundValue, bool);
  vtkBooleanMacro(UseBackgroundValue, bool);
  ///@}

  ///@{
  /**
   * Enable optional Gaussian smoothing before contouring for smoother results.
   * Default: false.
   */
  vtkSetMacro(SmoothContours, bool);
  vtkGetMacro(SmoothContours, bool);
  vtkBooleanMacro(SmoothContours, bool);
  ///@}

  ///@{
  /**
   * Standard deviation (sigma) for the Gaussian smoother, clamped to [0.01, 10.0].
   * Default: 0.5.
   */
  void SetSmoothStandardDeviation(double sigma);
  vtkGetMacro(SmoothStandardDeviation, double);
  ///@}

  ///@{
  /**
   * Enable / disable filled polygon generation on output port 1.
   * When true, closed contours are triangulated into filled polygon surfaces
   * using vtkContourTriangulator. Default: false.
   */
  vtkSetMacro(GenerateFilledPolygons, bool);
  vtkGetMacro(GenerateFilledPolygons, bool);
  vtkBooleanMacro(GenerateFilledPolygons, bool);
  ///@}

  ///@{
  /**
   * Enable / disable debounce for the computation pipeline.
   * When enabled, rapid successive calls to Update() within the debounce
   * interval will unconditionally reuse cached results from the previous
   * computation — even if the input data or filter parameters have changed.
   * A real recomputation only occurs on the first Update() call after the
   * debounce window has elapsed, ensuring the final result is always
   * up-to-date. This is useful when the filter is driven by interactive
   * events (e.g., mouse-driven reslicing) that may trigger updates faster
   * than needed. Default: false.
   *
   * @warning During the debounce window the output may be stale (reflecting
   * the previous input/parameters). Only enable this when temporary staleness
   * is acceptable (e.g., interactive preview scenarios).
   */
  vtkSetMacro(EnableDebounce, bool);
  vtkGetMacro(EnableDebounce, bool);
  vtkBooleanMacro(EnableDebounce, bool);
  ///@}

  ///@{
  /**
   * Debounce interval in milliseconds. When EnableDebounce is true,
   * RequestData will skip computation and reuse cached outputs if fewer
   * than DebounceInterval milliseconds have elapsed since the last
   * successful computation. Clamped to [1, 10000]. Default: 200.
   */
  void SetDebounceInterval(int intervalMs);
  vtkGetMacro(DebounceInterval, int);
  ///@}

protected:
  vtkImageLabelContourExtractor();
  ~vtkImageLabelContourExtractor() override;

  int FillInputPortInformation(int port, vtkInformation* info) override;
  int FillOutputPortInformation(int port, vtkInformation* info) override;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkImageLabelContourExtractor(const vtkImageLabelContourExtractor&) = delete;
  void operator=(const vtkImageLabelContourExtractor&) = delete;

  // ── Phase 1 ─────────────────────────────────────────────────────────────
  /** Collect all unique scalar values using vtkSMPTools::For + vtkSMPThreadLocal. */
  std::vector<double> CollectUniqueLabelsSMP(vtkImageData* input);

  // ── Phase 2 ─────────────────────────────────────────────────────────────
  /**
   * Build L binary masks in a single parallel scan.
   * @param input       Source label image.
   * @param labels      Sorted list of labels to process.
   * @param labelIndex  O(1) map: encoded-label -> index in [0, L).
   * @return            Vector of L binary float images (one per label).
   */
  std::vector<vtkSmartPointer<vtkImageData>> BuildAllBinaryMasksSMP(
    vtkImageData* input,
    const std::vector<double>& labels,
    const std::unordered_map<int64_t, vtkIdType>& labelIndex);

  // ── Phase 3 ─────────────────────────────────────────────────────────────
  /** Extract contour from a single binary mask (called in parallel per label). */
  vtkSmartPointer<vtkPolyData> ExtractContourFromMask(vtkImageData* binaryMask);

  // ── Phase 3b ────────────────────────────────────────────────────────────
  /** Generate filled polygon from a contour polydata (called in parallel per label). */
  vtkSmartPointer<vtkPolyData> GenerateFilledPolygonFromContour(vtkPolyData* contour);

  // ── Parameters ──────────────────────────────────────────────────────────
  double BackgroundValue;
  bool UseBackgroundValue;
  bool SmoothContours;
  double SmoothStandardDeviation;
  bool GenerateFilledPolygons;
  bool EnableDebounce;
  int DebounceInterval;

  // ── Debounce state ────────────────────────────────────────────────────
  /** Timestamp of the last successful computation (used for debounce logic). */
  std::chrono::steady_clock::time_point LastComputeTime;
  /** Whether cached outputs from a previous computation are available. */
  bool HasCachedOutput;
  /** Cached contour output from the last successful computation. */
  vtkSmartPointer<vtkMultiBlockDataSet> CachedContourOutput;
  /** Cached filled polygon output from the last successful computation. */
  vtkSmartPointer<vtkMultiBlockDataSet> CachedFilledOutput;
};

#endif // vtkImageLabelContourExtractor_h
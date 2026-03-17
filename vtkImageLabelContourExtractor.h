#ifndef vtkImageLabelContourExtractor_h
#define vtkImageLabelContourExtractor_h

#include <vtkMultiBlockDataSetAlgorithm.h>
#include <vtkSmartPointer.h>

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
 * iso-contours for every unique label.  The output is a vtkMultiBlockDataSet
 * where each block is a vtkPolyData representing the contour(s) of one label.
 *
 * ## 4-phase parallel pipeline (vtkSMPTools)
 *
 * - **Phase 1** – Parallel label collection using vtkSMPThreadLocal.
 * - **Phase 2** – Single-pass O(N) parallel binary-mask generation.
 * - **Phase 3** – Per-label parallel contour extraction (independent VTK pipelines).
 * - **Phase 4** – Serial assembly into vtkMultiBlockDataSet.
 *
 * Each output vtkPolyData carries a vtkDoubleArray named "LabelValue" in its
 * FieldData, and the corresponding block is named "Label_<intValue>".
 *
 * ### Typical usage
 * @code
 *   auto extractor = vtkSmartPointer<vtkImageLabelContourExtractor>::New();
 *   extractor->SetInputData(sliceImage);   // 2D vtkImageData
 *   extractor->SetBackgroundValue(0.0);    // skip background
 *   extractor->SmoothContoursOn();         // optional Gaussian smoothing
 *   extractor->Update();
 *   vtkMultiBlockDataSet* output = extractor->GetOutput();
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

protected:
  vtkImageLabelContourExtractor();
  ~vtkImageLabelContourExtractor() override;

  int FillInputPortInformation(int port, vtkInformation* info) override;
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
   * @param labelIndex  O(1) map: encoded-label → index in [0, L).
   * @return            Vector of L binary float images (one per label).
   */
  std::vector<vtkSmartPointer<vtkImageData>> BuildAllBinaryMasksSMP(
    vtkImageData* input,
    const std::vector<double>& labels,
    const std::unordered_map<int64_t, vtkIdType>& labelIndex);

  // ── Phase 3 ─────────────────────────────────────────────────────────────
  /** Extract contour from a single binary mask (called in parallel per label). */
  vtkSmartPointer<vtkPolyData> ExtractContourFromMask(vtkImageData* binaryMask);

  // ── Parameters ──────────────────────────────────────────────────────────
  double BackgroundValue;
  bool UseBackgroundValue;
  bool SmoothContours;
  double SmoothStandardDeviation;
};

#endif // vtkImageLabelContourExtractor_h

#include "vtkImageLabelContourExtractor.h"

#include <vtkCleanPolyData.h>
#include <vtkCompositeDataSet.h>
#include <vtkContourFilter.h>
#include <vtkContourTriangulator.h>
#include <vtkDataArray.h>
#include <vtkDataObject.h>
#include <vtkDoubleArray.h>
#include <vtkFieldData.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkImageGaussianSmooth.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSMPThreadLocal.h>
#include <vtkSMPTools.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

vtkStandardNewMacro(vtkImageLabelContourExtractor);

// ============================================================================
//  Construction / destruction
// ============================================================================
vtkImageLabelContourExtractor::vtkImageLabelContourExtractor()
  : BackgroundValue(0.0)
  , UseBackgroundValue(true)
  , SmoothContours(false)
  , SmoothStandardDeviation(0.5)
  , GenerateFilledPolygons(false)
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(2);
}
vtkImageLabelContourExtractor::~vtkImageLabelContourExtractor() = default;

// ============================================================================
//  PrintSelf
// ============================================================================
void vtkImageLabelContourExtractor::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "BackgroundValue: " << this->BackgroundValue << "\n";
  os << indent << "UseBackgroundValue: " << (this->UseBackgroundValue ? "true" : "false") << "\n";
  os << indent << "SmoothContours: " << (this->SmoothContours ? "true" : "false") << "\n";
  os << indent << "SmoothStandardDeviation: " << this->SmoothStandardDeviation << "\n";
  os << indent << "GenerateFilledPolygons: " << (this->GenerateFilledPolygons ? "true" : "false") << "\n";
}

// ============================================================================
//  Parameter setters
// ============================================================================
void vtkImageLabelContourExtractor::SetSmoothStandardDeviation(double sigma)
{
  double clamped = std::max(0.01, std::min(10.0, sigma));
  if (clamped != this->SmoothStandardDeviation)
  {
    this->SmoothStandardDeviation = clamped;
    this->Modified();
  }
}

// ============================================================================
//  Port information
// ============================================================================
int vtkImageLabelContourExtractor::FillInputPortInformation(int port, vtkInformation* info)
{
  if (port == 0)
  {
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkImageData");
    return 1;
  }
  return 0;
}

int vtkImageLabelContourExtractor::FillOutputPortInformation(int port, vtkInformation* info)
{
  if (port == 0 || port == 1)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMultiBlockDataSet");
    return 1;
  }
  return 0;
}

// ============================================================================
//  Phase 1 – Parallel label collection
// ============================================================================
std::vector<double> vtkImageLabelContourExtractor::CollectUniqueLabelsSMP(vtkImageData* input)
{
  vtkDataArray* scalars = input->GetPointData()->GetScalars();
  if (!scalars)
  {
    vtkErrorMacro("Input image has no scalar data.");
    return {};
  }

  const vtkIdType numPixels = scalars->GetNumberOfTuples();

  // Each thread accumulates unique labels into its own std::set – no contention.
  vtkSMPThreadLocal<std::set<double>> tlSets;

  vtkSMPTools::For(0, numPixels, [&](vtkIdType begin, vtkIdType end) {
    std::set<double>& localSet = tlSets.Local();
    for (vtkIdType i = begin; i < end; ++i)
    {
      localSet.insert(scalars->GetTuple1(i));
    }
  });

  // Reduce: merge all thread-local sets into one global sorted vector.
  std::set<double> global;
  for (auto& s : tlSets)
  {
    global.insert(s.begin(), s.end());
  }

  return std::vector<double>(global.begin(), global.end());
}

// ============================================================================
//  Phase 2 – Single-pass O(N) parallel binary-mask generation
// ============================================================================
// Encode a double label value to a reproducible integer key.
// We use bit-cast to int64 so that the same double always maps to the same key.
static inline int64_t EncodeLabel(double v)
{
  int64_t bits;
  std::memcpy(&bits, &v, sizeof(bits));
  return bits;
}

std::vector<vtkSmartPointer<vtkImageData>>
vtkImageLabelContourExtractor::BuildAllBinaryMasksSMP(
  vtkImageData* input,
  const std::vector<double>& labels,
  const std::unordered_map<int64_t, vtkIdType>& labelIndex)
{
  const vtkIdType numLabels = static_cast<vtkIdType>(labels.size());
  vtkDataArray* scalars = input->GetPointData()->GetScalars();
  const vtkIdType numPixels = scalars->GetNumberOfTuples();

  // Pre-allocate L float arrays, zero-initialised.
  // We store raw pointers for fast inner-loop access.
  std::vector<vtkSmartPointer<vtkFloatArray>> maskArrays(numLabels);
  std::vector<float*> maskPtrs(numLabels);

  for (vtkIdType li = 0; li < numLabels; ++li)
  {
    auto arr = vtkSmartPointer<vtkFloatArray>::New();
    arr->SetName("BinaryMask");
    arr->SetNumberOfTuples(numPixels);
    std::memset(arr->GetPointer(0), 0, static_cast<size_t>(numPixels) * sizeof(float));
    maskArrays[li] = arr;
    maskPtrs[li] = arr->GetPointer(0);
  }

  // Single parallel pass: each pixel index is processed by exactly one thread,
  // which writes to maskPtrs[labelIdx][i] – fully lock-free.
  vtkSMPTools::For(0, numPixels, [&](vtkIdType begin, vtkIdType end) {
    for (vtkIdType i = begin; i < end; ++i)
    {
      const double v = scalars->GetTuple1(i);
      auto it = labelIndex.find(EncodeLabel(v));
      if (it != labelIndex.end())
      {
        maskPtrs[it->second][i] = 1.0f;
      }
    }
  });

  // Wrap each array into a vtkImageData with the same geometry as the input.
  int dims[3];
  double spacing[3], origin[3];
  input->GetDimensions(dims);
  input->GetSpacing(spacing);
  input->GetOrigin(origin);

  std::vector<vtkSmartPointer<vtkImageData>> masks(numLabels);
  for (vtkIdType li = 0; li < numLabels; ++li)
  {
    auto img = vtkSmartPointer<vtkImageData>::New();
    img->SetDimensions(dims);
    img->SetSpacing(spacing);
    img->SetOrigin(origin);
    img->GetPointData()->SetScalars(maskArrays[li]);
    masks[li] = img;
  }

  return masks;
}

// ============================================================================
//  Phase 3 – Per-label contour extraction
// ============================================================================
vtkSmartPointer<vtkPolyData> vtkImageLabelContourExtractor::ExtractContourFromMask(
  vtkImageData* binaryMask)
{
  vtkAlgorithmOutput* inputPort = nullptr;
  vtkNew<vtkImageGaussianSmooth> smoother;

  if (this->SmoothContours)
  {
    const double sd = this->SmoothStandardDeviation;
    smoother->SetInputData(binaryMask);
    smoother->SetStandardDeviations(sd, sd, 0.0);
    // Radius factor ~2× sigma is a good trade-off between quality and speed.
    smoother->SetRadiusFactors(2.0, 2.0, 0.0);
    smoother->SetDimensionality(2);
    smoother->Update();
    inputPort = smoother->GetOutputPort();
  }

  // vtkContourFilter → Marching Squares for 2D images.
  vtkNew<vtkContourFilter> contour;
  if (this->SmoothContours)
  {
    contour->SetInputConnection(inputPort);
  }
  else
  {
    contour->SetInputData(binaryMask);
  }
  contour->SetValue(0, 0.5);
  contour->Update();

  // Join disconnected line segments into continuous polylines.
  vtkNew<vtkStripper> stripper;
  stripper->SetInputConnection(contour->GetOutputPort());
  stripper->SetJoinContiguousSegments(true);
  stripper->Update();

  // Remove duplicate points.
  vtkNew<vtkCleanPolyData> cleaner;
  cleaner->SetInputConnection(stripper->GetOutputPort());
  cleaner->SetTolerance(1e-6);
  cleaner->Update();

  auto result = vtkSmartPointer<vtkPolyData>::New();
  result->DeepCopy(cleaner->GetOutput());
  return result;
}

// ============================================================================
//  Phase 3b – Per-label filled polygon generation
// ============================================================================
vtkSmartPointer<vtkPolyData> vtkImageLabelContourExtractor::GenerateFilledPolygonFromContour(
  vtkPolyData* contour)
{
  if (!contour || contour->GetNumberOfCells() == 0)
  {
    auto empty = vtkSmartPointer<vtkPolyData>::New();
    return empty;
  }

  vtkNew<vtkContourTriangulator> triangulator;
  triangulator->SetInputData(contour);
  triangulator->Update();

  auto result = vtkSmartPointer<vtkPolyData>::New();
  result->DeepCopy(triangulator->GetOutput());
  return result;
}

// ============================================================================
//  RequestData – orchestrates all phases
// ============================================================================
int vtkImageLabelContourExtractor::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  // ── Obtain input / output ────────────────────────────────────────────────
vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
vtkImageData* input =
vtkImageData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

  if (!input)
  {
    vtkErrorMacro("Input is not a vtkImageData.");
    return 0;
  }

  int dims[3];
  input->GetDimensions(dims);
  if (dims[0] > 1 && dims[1] > 1 && dims[2] > 1)
  {
    vtkWarningMacro("Input appears to be 3D (" << dims[0] << "x" << dims[1] << "x" << dims[2]
      << "). This filter is optimized for 2D images.");
  }

  // Output port 0: contours (always)
vtkInformation* outInfo0 = outputVector->GetInformationObject(0);
vtkMultiBlockDataSet* contourOutput =
vtkMultiBlockDataSet::SafeDownCast(outInfo0->Get(vtkDataObject::DATA_OBJECT()));

  if (!contourOutput)
  {
    vtkErrorMacro("Output port 0 is not a vtkMultiBlockDataSet.");
    return 0;
  }

  // Output port 1: filled polygons (only used when GenerateFilledPolygons is true)
vtkInformation* outInfo1 = outputVector->GetInformationObject(1);
vtkMultiBlockDataSet* filledOutput =
vtkMultiBlockDataSet::SafeDownCast(outInfo1->Get(vtkDataObject::DATA_OBJECT()));

  if (!filledOutput)
  {
    vtkErrorMacro("Output port 1 is not a vtkMultiBlockDataSet.");
    return 0;
  }

  // ── Phase 1: Parallel label collection ──────────────────────────────────
  std::vector<double> labels = this->CollectUniqueLabelsSMP(input);

  if (labels.empty())
  {
    vtkWarningMacro("No labels found in the input image.");
    contourOutput->SetNumberOfBlocks(0);
    filledOutput->SetNumberOfBlocks(0);
    return 1;
  }

  // Remove background label if requested.
  if (this->UseBackgroundValue)
  {
    labels.erase(
      std::remove(labels.begin(), labels.end(), this->BackgroundValue),
      labels.end());
  }

  if (labels.empty())
  {
    vtkWarningMacro("All labels were excluded (background only).");
    contourOutput->SetNumberOfBlocks(0);
    filledOutput->SetNumberOfBlocks(0);
    return 1;
  }

  const vtkIdType numLabels = static_cast<vtkIdType>(labels.size());

  // Build O(1) lookup: encoded-label → index.
  std::unordered_map<int64_t, vtkIdType> labelIndex;
  labelIndex.reserve(static_cast<size_t>(numLabels));
  for (vtkIdType li = 0; li < numLabels; ++li)
  {
    labelIndex[EncodeLabel(labels[static_cast<size_t>(li)])] = li;
  }

  // ── Phase 2: Single-pass parallel binary-mask generation ────────────────
  std::vector<vtkSmartPointer<vtkImageData>> masks =
    this->BuildAllBinaryMasksSMP(input, labels, labelIndex);

  // ── Phase 3: Parallel contour extraction – one pipeline per label ────────
  // Pre-allocate result slots so each thread can write to its own index
  // without any synchronisation.
  std::vector<vtkSmartPointer<vtkPolyData>> contours(static_cast<size_t>(numLabels));

  vtkSMPTools::For(0, numLabels, [&](vtkIdType begin, vtkIdType end) {
    for (vtkIdType li = begin; li < end; ++li)
    {
      contours[static_cast<size_t>(li)] =
        this->ExtractContourFromMask(masks[static_cast<size_t>(li)]);
    }
  });

  // ── Phase 3b: Parallel filled polygon generation (if enabled) ────────────
  std::vector<vtkSmartPointer<vtkPolyData>> filledPolygons(static_cast<size_t>(numLabels));

  if (this->GenerateFilledPolygons)
  {
    vtkSMPTools::For(0, numLabels, [&](vtkIdType begin, vtkIdType end) {
      for (vtkIdType li = begin; li < end; ++li)
      {
        filledPolygons[static_cast<size_t>(li)] =
          this->GenerateFilledPolygonFromContour(contours[static_cast<size_t>(li)]);
      }
    });
  }

  // ── Phase 4: Serial assembly into vtkMultiBlockDataSet(s) ────────────────
  // vtkMultiBlockDataSet::SetBlock is NOT thread-safe, so this phase is serial.

  // --- Port 0: Contours (always) ---
  contourOutput->SetNumberOfBlocks(static_cast<unsigned int>(numLabels));

  for (vtkIdType li = 0; li < numLabels; ++li)
  {
    const double labelValue = labels[static_cast<size_t>(li)];
    vtkPolyData* poly = contours[static_cast<size_t>(li)];

    // Attach the actual pixel value as FieldData on the polydata.
    vtkNew<vtkDoubleArray> labelArr;
    labelArr->SetName("LabelValue");
    labelArr->SetNumberOfTuples(1);
    labelArr->SetValue(0, labelValue);
    poly->GetFieldData()->AddArray(labelArr);

    contourOutput->SetBlock(static_cast<unsigned int>(li), poly);

    // Set human-readable block name.
    std::string blockName = "Label_" + std::to_string(static_cast<int>(labelValue));
    contourOutput->GetMetaData(static_cast<unsigned int>(li))
      ->Set(vtkCompositeDataSet::NAME(), blockName.c_str());
  }

  // --- Port 1: Filled polygons (only if enabled) ---
  if (this->GenerateFilledPolygons)
  {
    filledOutput->SetNumberOfBlocks(static_cast<unsigned int>(numLabels));

    for (vtkIdType li = 0; li < numLabels; ++li)
    {
      const double labelValue = labels[static_cast<size_t>(li)];
      vtkPolyData* filledPoly = filledPolygons[static_cast<size_t>(li)];

      // Attach the actual pixel value as FieldData on the filled polydata.
      vtkNew<vtkDoubleArray> labelArr;
      labelArr->SetName("LabelValue");
      labelArr->SetNumberOfTuples(1);
      labelArr->SetValue(0, labelValue);
      filledPoly->GetFieldData()->AddArray(labelArr);

      filledOutput->SetBlock(static_cast<unsigned int>(li), filledPoly);

      // Set human-readable block name.
      std::string blockName = "Label_" + std::to_string(static_cast<int>(labelValue));
      filledOutput->GetMetaData(static_cast<unsigned int>(li))
        ->Set(vtkCompositeDataSet::NAME(), blockName.c_str());
    }
  }
  else
  {
    filledOutput->SetNumberOfBlocks(0);
  }

  return 1;
}
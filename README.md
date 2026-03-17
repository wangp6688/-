# vtkImageLabelContourExtractor

High-performance parallel contour extraction filter for 2D label images, based on VTK 9.1+.

## Features

- Extracts contours for every unique pixel value from a 2D `vtkImageData`
- Multi-threaded using `vtkSMPTools` (supports TBB/STDThread/OpenMP backends)
- 4-phase parallel pipeline for maximum throughput
- Outputs `vtkMultiBlockDataSet` with one `vtkPolyData` per label
- Optional Gaussian smoothing for smoother contours
- Configurable background value exclusion

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Usage

```cpp
auto extractor = vtkSmartPointer<vtkImageLabelContourExtractor>::New();
extractor->SetInputData(sliceImage);
extractor->SetBackgroundValue(0);
extractor->Update();
vtkMultiBlockDataSet* output = extractor->GetOutput();
```

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

---

## CurveVectorTransport

Header-only utility (`CurveVectorTransport.h`) that **parallel-transports a set
of direction vectors along a discrete 3-D curve** (Bishop-frame / rotation-minimising
frame propagation).  Uses `vtkVector3d` for coordinates and `vtkMath` for all
vector operations.

### Problem statement

Given:

| Input | Description |
|-------|-------------|
| Ordered **point set** | Defines the curve as a 3-D polyline (`std::vector<vtkVector3d>`). |
| **Start point** | A point on (or near) the curve. The origin of the local frame. |
| **Direction vectors** | Vectors lying in the plane whose normal is the curve tangent at the start point (i.e. perpendicular to the tangent). |
| **End point** | Target point on (or near) the curve. |

Output: the direction vectors *transported* to the end point — they remain
perpendicular to the tangent at every step and accumulate the minimum possible
twist (parallel transport).

### Algorithm

At each polyline edge the accumulated frame is rotated by the unique,
smallest-angle rotation that maps the current edge tangent onto the next one
(Rodrigues' formula around the cross-product axis).  This is the discrete
analogue of parallel transport.

```
For each edge i → i+1 (walking from startIndex to endIndex):
    diff    = P[i+1] - P[i]
    T_next  = Normalize( diff )                       // copy, then vtkMath::Normalize in-place
    axis    = vtkMath::Cross( T_current, T_next )     // rotation axis
    sinA    = vtkMath::Norm( axis )
    cosA    = vtkMath::Dot( T_current, T_next )
    Rotate every direction vector (Rodrigues' formula)
    T_current = T_next
```

Any component of the input direction vectors along the start tangent is
removed before transport begins, enforcing the perpendicularity constraint.

### Quick-start

```cpp
#include "CurveVectorTransport.h"

// Define the curve.
std::vector<vtkVector3d> pts = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.5, 0.0},
    {3.0, 1.0, 0.0}
};

// Direction vectors at the start, perpendicular to the tangent there.
// (tangent at index 0 ≈ {1,0,0}, so {0,1,0} and {0,0,1} are valid)
std::vector<vtkVector3d> dirs = { {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };

// Transport to the end point — single function call.
std::vector<vtkVector3d> result = TransportVectorsAlongCurve(
    pts,
    vtkVector3d(0.0, 0.0, 0.0),   // start point
    dirs,
    vtkVector3d(3.0, 1.0, 0.0));  // end point

// result[0] and result[1] are the transported vectors at the end.
```

### API

```cpp
std::vector<vtkVector3d> TransportVectorsAlongCurve(
    const std::vector<vtkVector3d>& curvePoints,
    const vtkVector3d&              startPoint,
    const std::vector<vtkVector3d>& directions,
    const vtkVector3d&              endPoint);
```

| Parameter | Description |
|-----------|-------------|
| `curvePoints` | Ordered 3-D polyline (≥ 2 points). |
| `startPoint` | Origin of the transport (matched to nearest curve point). |
| `directions` | Direction vectors at the start (perpendicular to the tangent). |
| `endPoint` | Target point (matched to nearest curve point). |
| **Return** | Transported vectors, one per input direction. |

Throws `std::invalid_argument` if the curve has fewer than 2 points or if
`directions` is empty.

### Notes

- Start / end points are matched to the **nearest** point in the set.
- Transport works in **both directions** along the curve (end index may be
  smaller than start index).
- Degenerate cases (zero-length edges, anti-parallel consecutive tangents) are
  handled gracefully.
- Requires VTK (`vtkMath`, `vtkVector`). Header-only, C++14.


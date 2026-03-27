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
frame propagation).  No VTK or any other external dependency is required.

### Problem statement

Given:

| Input | Description |
|-------|-------------|
| Ordered **point set** | Defines the curve as a 3-D polyline. |
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
    T_next  = normalize( P[i+1] - P[i] )          // next edge tangent
    axis    = T_current × T_next                   // rotation axis
    angle   = arccos( T_current · T_next )         // rotation angle
    Rotate every direction vector around axis by angle   (Rodrigues)
    T_current = T_next
```

Any component of the input direction vectors along the start tangent is
removed before transport begins, enforcing the perpendicularity constraint.

### Quick-start

```cpp
#include "CurveVectorTransport.h"

using Vec3 = CurveVectorTransport::Vec3;

// Define the curve.
std::vector<Vec3> pts = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.5, 0.0},
    {3.0, 1.0, 0.0}
};

// Direction vectors at the start, perpendicular to the tangent there.
// (tangent at index 0 ≈ {1,0,0}, so {0,1,0} and {0,0,1} are valid)
std::vector<Vec3> dirs = { {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };

CurveVectorTransport cvt;
cvt.SetCurvePoints(pts);
cvt.SetStartPoint({0.0, 0.0, 0.0});
cvt.SetDirections(dirs);

// Transport to the end point.
std::vector<Vec3> result = cvt.TransportToEndPoint({3.0, 1.0, 0.0});
// result[0] and result[1] are the transported vectors at the end.
```

### API

| Method | Description |
|--------|-------------|
| `SetCurvePoints(pts)` | Set the ordered 3-D polyline. |
| `SetStartPoint(pt)` | Set the start point (matched to nearest curve point). |
| `SetDirections(dirs)` | Set the direction vectors at the start. |
| `TransportToEndPoint(pt)` → `vector<Vec3>` | Run the transport; returns one result vector per input direction. |

### Notes

- Start / end points are matched to the **nearest** point in the set.
- Transport works in **both directions** along the curve (end index may be
  smaller than start index).
- Degenerate cases (zero-length edges, anti-parallel consecutive tangents) are
  handled gracefully.
- Header-only, standard **C++14**, no external dependencies.

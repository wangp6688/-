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

## vtkCameraRotateHandleSource

A `vtkPolyDataAlgorithm` that generates the rotate-handle shape (from
`rotate_handle.svg`) as a `vtkPolyData` — with **no runtime SVG parsing**.

The 2 SVG paths were tessellated into 177 2-D sample points at code-generation
time (cubic Bézier curves subdivided into 32 segments).  The resulting
coordinate arrays are baked directly into `vtkCameraRotateHandleSource.cxx`.

The shape is placed in 3-D space via three parameters:

| Parameter   | Default      | Description |
|-------------|--------------|-------------|
| `Center`    | (0, 0, 0)    | World-space origin of the shape. |
| `Normal`    | (0, 0, 1)    | Normal of the plane the shape lies in. |
| `Direction` | (1, 0, 0)    | In-plane direction mapped to the SVG +X axis (projected onto the Normal plane automatically). |
| `Scale`     | 1.0          | Uniform scale applied to all shape coordinates. |

Two output modes can be toggled independently:

| Mode              | Default | Description |
|-------------------|---------|-------------|
| `GeneratePolyline`| on      | Closed polyline cells (outline). |
| `GeneratePolygon` | on      | Filled polygon cells. |

### Quick-start

```cpp
#include "vtkCameraRotateHandleSource.h"

auto src = vtkSmartPointer<vtkCameraRotateHandleSource>::New();
src->SetCenter(0.0, 0.0, 0.0);
src->SetNormal(0.0, 0.0, 1.0);
src->SetDirection(1.0, 0.0, 0.0);
src->SetScale(50.0);
src->Update();
vtkPolyData* pd = src->GetOutput();
```

### Build

The class is compiled as a static library (`vtkCameraRotateHandleSource`) alongside
`vtkImageLabelContourExtractor` via the provided `CMakeLists.txt`.
A standalone interactive example is built as `Example_RotateHandle`:

```bash
mkdir build && cd build
cmake ..
make
./Example_RotateHandle
```

### How the geometry was generated

A Python script (`/tmp/gen_vtk_source.py`) parsed `rotate_handle.svg` offline:

1. Each SVG `<path d="...">` was tokenised into M / L / C / Z commands.
2. Cubic Bézier (`C`) segments were tessellated at 32 equidistant parameter
   steps using de-Casteljau evaluation.
3. All coordinates were normalised to the range `[-0.5, +0.5]` (centred on the
   SVG viewBox) and the SVG Y-axis was flipped to match the conventional
   right-handed +Y-up convention.
4. The resulting arrays were written into `vtkCameraRotateHandleSource.cxx`.

---

## vtkCameraRotateAxisSource

A `vtkPolyDataAlgorithm` that generates the rotate-axis shape (from
`rotate_axis.svg`) as a `vtkPolyData` — with **no runtime SVG parsing**.

The 2 SVG sub-paths were tessellated into 138 2-D sample points at
code-generation time (cubic Bézier curves subdivided into 32 segments).
The resulting coordinate arrays are baked directly into
`vtkCameraRotateAxisSource.cxx`.

The shape is placed in 3-D space via three parameters:

| Parameter   | Default      | Description |
|-------------|--------------|-------------|
| `Center`    | (0, 0, 0)    | World-space origin of the shape. |
| `Normal`    | (0, 0, 1)    | Normal of the plane the shape lies in. |
| `Direction` | (0, 1, 0)    | In-plane direction mapped to the SVG +Y axis (projected onto the Normal plane automatically). |
| `Scale`     | 1.0          | Uniform scale applied to all shape coordinates. |

Two output modes can be toggled independently:

| Mode              | Default | Description |
|-------------------|---------|-------------|
| `GeneratePolyline`| on      | Closed polyline cells (outline). |
| `GeneratePolygon` | on      | Filled polygon cells. |

### Quick-start

```cpp
#include "vtkCameraRotateAxisSource.h"

auto src = vtkSmartPointer<vtkCameraRotateAxisSource>::New();
src->SetCenter(0.0, 0.0, 0.0);
src->SetNormal(0.0, 0.0, 1.0);
src->SetDirection(0.0, 1.0, 0.0);
src->SetScale(50.0);
src->Update();
vtkPolyData* pd = src->GetOutput();
```

### Build

The class is compiled as a static library (`vtkCameraRotateAxisSource`) alongside
`vtkImageLabelContourExtractor` and `vtkCameraRotateHandleSource` via the provided
`CMakeLists.txt`.  A standalone interactive example is built as `Example_RotateAxis`:

```bash
mkdir build && cd build
cmake ..
make
./Example_RotateAxis
```

### How the geometry was generated

A Python script parsed `rotate_axis.svg` offline:

1. The SVG `<path d="...">` was tokenised into M / L / C / Z commands.
2. The path contained 2 sub-paths (separated by the second `M` command).
3. Cubic Bézier (`C`) segments were tessellated at 32 equidistant parameter
   steps using de-Casteljau evaluation.
4. All coordinates were normalised to the range `[-0.5, +0.5]` (centred on the
   SVG viewBox `0 0 323 16`) and the SVG Y-axis was flipped to match the
   conventional right-handed +Y-up convention.
5. The resulting arrays were written into `vtkCameraRotateAxisSource.cxx`.



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


cmake_minimum_required(VERSION 3.12)
project(ImageLabelContourExtractor CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Find VTK 9.1+ ────────────────────────────────────────────────────────────
find_package(VTK 9.1 REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  CommonExecutionModel
  FiltersCore
  FiltersGeneral
  ImagingGeneral
  InteractionStyle
  RenderingCore
  RenderingOpenGL2
  RenderingFreeType
)

# ── Library: vtkImageLabelContourExtractor ───────────────────────────────────
add_library(vtkImageLabelContourExtractor STATIC
  vtkImageLabelContourExtractor.h
  vtkImageLabelContourExtractor.cxx
)

# ── Library: vtkCameraRotateHandleSource ────────────────────────────────────────────
add_library(vtkCameraRotateHandleSource STATIC
  vtkCameraRotateHandleSource.h
  vtkCameraRotateHandleSource.cxx
)

# ── Library: vtkCameraRotateAxisSource ──────────────────────────────────────────────
add_library(vtkCameraRotateAxisSource STATIC
  vtkCameraRotateAxisSource.h
  vtkCameraRotateAxisSource.cxx
)

target_include_directories(vtkImageLabelContourExtractor
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(vtkImageLabelContourExtractor
  PUBLIC ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS vtkImageLabelContourExtractor
  MODULES ${VTK_LIBRARIES}
)

target_include_directories(vtkCameraRotateHandleSource
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(vtkCameraRotateHandleSource
  PUBLIC ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS vtkCameraRotateHandleSource
  MODULES ${VTK_LIBRARIES}
)

target_include_directories(vtkCameraRotateAxisSource
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(vtkCameraRotateAxisSource
  PUBLIC ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS vtkCameraRotateAxisSource
  MODULES ${VTK_LIBRARIES}
)

# ── Example executable ────────────────────────────────────────────────────────
add_executable(Example Example_Usage.cxx)

target_link_libraries(Example
  PRIVATE vtkImageLabelContourExtractor
          ${VTK_LIBRARIES}
)

# ── RotateHandle example executable ─────────────────────────────────────────
add_executable(Example_RotateHandle Example_RotateHandle.cxx)

target_link_libraries(Example_RotateHandle
  PRIVATE vtkCameraRotateHandleSource
          ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS Example
  MODULES ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS Example_RotateHandle
  MODULES ${VTK_LIBRARIES}
)

# ── RotateAxis example executable ───────────────────────────────────────────
add_executable(Example_RotateAxis Example_RotateAxis.cxx)

target_link_libraries(Example_RotateAxis
  PRIVATE vtkCameraRotateAxisSource
          ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS Example_RotateAxis
  MODULES ${VTK_LIBRARIES}
)

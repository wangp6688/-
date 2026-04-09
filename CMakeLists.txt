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

# ── Library: vtkDSARotateHandleSource ────────────────────────────────────────────
add_library(vtkDSARotateHandleSource STATIC
  vtkDSARotateHandleSource.h
  vtkDSARotateHandleSource.cxx
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

target_include_directories(vtkDSARotateHandleSource
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(vtkDSARotateHandleSource
  PUBLIC ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS vtkDSARotateHandleSource
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
  PRIVATE vtkDSARotateHandleSource
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

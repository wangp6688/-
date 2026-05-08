#include "vtkOutlinedTextActor.h"

#include <vtkCoordinate.h>
#include <vtkObjectFactory.h>
#include <vtkTextProperty.h>
#include <vtkViewport.h>
#include <vtkWindow.h>

vtkStandardNewMacro(vtkOutlinedTextActor);

namespace
{
constexpr int BorderOffsets[8][2] = { { -1, 0 }, { 1, 0 }, { 0, 1 }, { 0, -1 },
                                      { -1, 1 }, { 1, 1 }, { -1, -1 }, { 1, -1 } };
}

vtkOutlinedTextActor::vtkOutlinedTextActor()
  : OutlineOpacity(0.85)
  , OutlineOffset(1)
  , OutlineVisibility(true)
{
  this->OutlineColor[0] = 0.1;
  this->OutlineColor[1] = 0.1;
  this->OutlineColor[2] = 0.1;

  for (auto& actor : this->BorderActors)
  {
    actor = vtkSmartPointer<vtkTextActor>::New();
    actor->SetTextScaleModeToNone();
    actor->GetTextProperty()->SetColor(this->OutlineColor);
    actor->GetTextProperty()->SetOpacity(this->OutlineOpacity);
    actor->SetVisibility(this->OutlineVisibility);
  }
}

void vtkOutlinedTextActor::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "OutlineColor: (" << this->OutlineColor[0] << ", " << this->OutlineColor[1] << ", "
     << this->OutlineColor[2] << ")\n";
  os << indent << "OutlineOpacity: " << this->OutlineOpacity << "\n";
  os << indent << "OutlineOffset: " << this->OutlineOffset << "\n";
  os << indent << "OutlineVisibility: " << (this->OutlineVisibility ? "On" : "Off") << "\n";
}

void vtkOutlinedTextActor::SyncOutlineText()
{
  const char* text = this->GetInput();
  vtkTextProperty* sourceProperty = this->GetTextProperty();

  for (auto& actor : this->BorderActors)
  {
    actor->SetInput(text ? text : "");
    actor->SetTextScaleMode(this->GetTextScaleMode());
    vtkTextProperty* borderProperty = actor->GetTextProperty();
    borderProperty->ShallowCopy(sourceProperty);
    borderProperty->SetColor(this->OutlineColor);
    borderProperty->SetOpacity(this->OutlineOpacity);
    actor->SetVisibility(this->OutlineVisibility && this->GetVisibility());
  }
}

void vtkOutlinedTextActor::SyncOutlinePositions(vtkViewport* viewport)
{
  if (!viewport)
  {
    return;
  }

  int* displayPosition = this->GetPositionCoordinate()->GetComputedDisplayValue(viewport);
  const int displayX = displayPosition[0];
  const int displayY = displayPosition[1];
  const int offset = this->OutlineOffset;

  for (int i = 0; i < 8; ++i)
  {
    this->BorderActors[i]->SetPosition(displayX + BorderOffsets[i][0] * offset,
                                       displayY + BorderOffsets[i][1] * offset);
  }
}

void vtkOutlinedTextActor::SyncOutlineActors(vtkViewport* viewport)
{
  this->SyncOutlineText();
  this->SyncOutlinePositions(viewport);
}

int vtkOutlinedTextActor::RenderOverlay(vtkViewport* viewport)
{
  this->SyncOutlineActors(viewport);

  int count = 0;
  if (this->OutlineVisibility)
  {
    for (auto& actor : this->BorderActors)
    {
      count += actor->RenderOverlay(viewport);
    }
  }
  count += this->Superclass::RenderOverlay(viewport);
  return count;
}

int vtkOutlinedTextActor::RenderOpaqueGeometry(vtkViewport* viewport)
{
  this->SyncOutlineActors(viewport);

  int count = 0;
  if (this->OutlineVisibility)
  {
    for (auto& actor : this->BorderActors)
    {
      count += actor->RenderOpaqueGeometry(viewport);
    }
  }
  count += this->Superclass::RenderOpaqueGeometry(viewport);
  return count;
}

int vtkOutlinedTextActor::RenderTranslucentPolygonalGeometry(vtkViewport* viewport)
{
  this->SyncOutlineActors(viewport);

  int count = 0;
  if (this->OutlineVisibility)
  {
    for (auto& actor : this->BorderActors)
    {
      count += actor->RenderTranslucentPolygonalGeometry(viewport);
    }
  }
  count += this->Superclass::RenderTranslucentPolygonalGeometry(viewport);
  return count;
}

vtkTypeBool vtkOutlinedTextActor::HasTranslucentPolygonalGeometry()
{
  int result = this->Superclass::HasTranslucentPolygonalGeometry();
  if (this->OutlineVisibility)
  {
    for (auto& actor : this->BorderActors)
    {
      result |= actor->HasTranslucentPolygonalGeometry();
    }
  }
  return result;
}

void vtkOutlinedTextActor::ReleaseGraphicsResources(vtkWindow* window)
{
  for (auto& actor : this->BorderActors)
  {
    actor->ReleaseGraphicsResources(window);
  }
  this->Superclass::ReleaseGraphicsResources(window);
}

#include "CaptionRepresentation.h"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkActor2D.h>
#include <vtkLineSource.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkViewport.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkProperty2D.h>

#include <shlobj.h>
#include <windows.h>
#include <QDir>

#include <vtkDashedPolyLine.h>
#include <VtkCommon.h>

using namespace mediBlade;
vtkStandardNewMacro(CaptionRepresentation);

void CaptionRepresentation::EnableShadow(bool bOn) {
    if (m_bShowShadow != bOn) {
        m_bShowShadow = bOn;
        // 同步所有描边文本的可见性
        for (int i = 0; i < 8; ++i) {
            m_borderActors[i]->SetVisibility(bOn);
        }
        this->Modified();  // 触发渲染更新
    }
}

QString CaptionRepresentation::GetWindowsFontsPath() {
    QString systemFontsPath("C:/Windows/Fonts");

    wchar_t path[MAX_PATH];
    if (SHGetFolderPathW(NULL, CSIDL_FONTS, NULL, 0, path) == S_OK) {
        systemFontsPath = QString::fromWCharArray(path);
    }
    return QDir(systemFontsPath).canonicalPath();
}

CaptionRepresentation::CaptionRepresentation()
    : m_pLine(vtkSmartPointer<vtkDashedPolyLine>::New()),
      m_pLineActor(vtkSmartPointer<vtkActor2D>::New()),
      m_pAttachmentCoordinate(vtkSmartPointer<vtkCoordinate>::New()),
      m_pVertexSource(vtkSmartPointer<vtkRegularPolygonSource>::New()),
      m_pVertexActor(vtkSmartPointer<vtkActor>::New()),
      m_pVertexMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
      m_bVertexVisible(false),
      m_vertexFixedPixelSize(3)
{
    // 初始化描边颜色
    m_borderColor[0] = 0.1;
    m_borderColor[1] = 0.1;
    m_borderColor[2] = 0.1;
    m_borderColorAlpha = 0.85;
    // 初始化8个方向的描边文本（上下左右+4个对角线）
    for (int i = 0; i < 8; ++i) {
        m_borderActors[i] = vtkSmartPointer<vtkTextActor>::New();
        m_borderActors[i]->SetTextScaleModeToNone();
        m_borderActors[i]->GetTextProperty()->SetColor(m_borderColor);
        m_borderActors[i]->GetTextProperty()->SetOpacity(m_borderColorAlpha);
        m_borderActors[i]->SetVisibility(m_bShowShadow);
    }

    this->m_pLine->setDashedLength(5.0);

    vtkSmartPointer<vtkPolyDataMapper2D> mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    mapper->SetInputConnection(this->m_pLine->GetOutputPort());
    this->m_pLineActor->SetMapper(mapper);

    this->m_pLineActor->GetProperty()->SetColor(1, 1, 1);

    this->m_pAttachmentCoordinate->SetCoordinateSystemToWorld();
    this->m_pAttachmentCoordinate->SetValue(0.0, 0.0, 0.0);

    // 初始化顶层文本（内容）
    this->TextActor->SetTextScaleModeToNone();
    this->SetPadding(0);
    this->SetShowBorder(vtkBorderRepresentation::BORDER_OFF);

    this->EnforceNormalizedViewportBoundsOn();

    // 顶点初始化：用30边正多边形模拟圆形（平面蓝点）
    m_pVertexSource->SetNumberOfSides(30);      // 30边→近似圆形
    m_pVertexSource->SetRadius(1.0);            // 初始半径（后续动态更新）
    m_pVertexSource->SetCenter(0.0, 0.0, 0.0);  // 初始中心点
    m_pVertexSource->SetNormal(0.0, 0.0, 1.0);  // 法线方向（Z轴，确保在2D平面）
    m_pVertexSource->Update();

    // 绑定顶点渲染管线
    m_pVertexMapper->SetInputConnection(m_pVertexSource->GetOutputPort());
    m_pVertexActor->SetMapper(m_pVertexMapper);
    m_pVertexActor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // 蓝色
    m_pVertexActor->SetVisibility(m_bVertexVisible);         // 默认隐藏
}

CaptionRepresentation::~CaptionRepresentation() {
}

void CaptionRepresentation::SyncShadowTextActors() {
    // 同步文本内容到8个描边文本
    const char* text = this->TextActor->GetInput();
    for (int i = 0; i < 8; ++i) {
        m_borderActors[i]->SetInput(text);
    }

    // 同步字体样式（保持与顶层文本一致）
    vtkTextProperty* srcProp = this->TextActor->GetTextProperty();
    for (int i = 0; i < 8; ++i) {
        vtkTextProperty* borderProp = m_borderActors[i]->GetTextProperty();
        borderProp->SetFontFamily(srcProp->GetFontFamily());
        borderProp->SetFontFile(srcProp->GetFontFile());
        borderProp->SetFontSize(srcProp->GetFontSize());  // 同字号
        borderProp->SetBold(srcProp->GetBold());
        borderProp->SetItalic(srcProp->GetItalic());
        borderProp->SetColor(m_borderColor);  // 描边色
        borderProp->SetOpacity(m_borderColorAlpha);
    }
    for (int i = 0; i < 8; ++i) {
        m_borderActors[i]->SetVisibility(m_bShowShadow);
    }
}

void CaptionRepresentation::SyncShadowTextPositions(vtkViewport* viewport) {
    if (!viewport)
        return;

    // 获取顶层文本的屏幕坐标
    double* pos = this->PositionCoordinate->GetValue();
    int* viewSize = viewport->GetSize();
    double displayX = pos[0] * viewSize[0];
    double displayY = pos[1] * viewSize[1];

    // 8个方向偏移（1px）
    // 0:左 1:右 2:上 3:下
    // 4:左上 5:右上 6:左下 7:右下
    m_borderActors[0]->SetPosition(displayX - m_borderOffset, displayY);
    m_borderActors[1]->SetPosition(displayX + m_borderOffset, displayY);
    m_borderActors[2]->SetPosition(displayX, displayY + m_borderOffset);
    m_borderActors[3]->SetPosition(displayX, displayY - m_borderOffset);
    m_borderActors[4]->SetPosition(displayX - m_borderOffset, displayY + m_borderOffset);
    m_borderActors[5]->SetPosition(displayX + m_borderOffset, displayY + m_borderOffset);
    m_borderActors[6]->SetPosition(displayX - m_borderOffset, displayY - m_borderOffset);
    m_borderActors[7]->SetPosition(displayX + m_borderOffset, displayY - m_borderOffset);
}

void CaptionRepresentation::UpdateLine(vtkViewport* v) {
    if (m_vecAnchors.empty()) {
        return;
    }
    double lfPoint[4] = {0}, urPoint[4] = {0}, anchorPoint[4] = {0};
    int *lfDPoint, *urDPoint, *anchorDPoint;
    lfDPoint = this->PositionCoordinate->GetComputedDisplayValue(v);
    urDPoint = this->Position2Coordinate->GetComputedDisplayValue(v);
    vtkVector3d closestWAnchor, edgeDPoint;
    double lineLength = VTK_DOUBLE_MAX;
    for (auto iter = m_vecAnchors.begin(); iter != m_vecAnchors.end(); iter++) {
        this->m_pAttachmentCoordinate->SetValue(iter->GetData());
        anchorDPoint = this->m_pAttachmentCoordinate->GetComputedDisplayValue(v);

        anchorPoint[0] = double(anchorDPoint[0]);
        anchorPoint[1] = double(anchorDPoint[1]);
        anchorPoint[2] = 0.0;

        lfPoint[0] = (double)lfDPoint[0];
        lfPoint[1] = (double)lfDPoint[1];
        lfPoint[2] = anchorPoint[2];

        urPoint[0] = (double)urDPoint[0];
        urPoint[1] = (double)urDPoint[1];
        urPoint[2] = lfPoint[2];

        vtkVector3d tempPoint = FindClosestPoint(anchorPoint, lfPoint, urPoint);
        double length = vtkMath::Distance2BetweenPoints(tempPoint.GetData(), anchorPoint);
        if (length < lineLength) {
            lineLength = length;
            closestWAnchor = *iter;
            edgeDPoint = tempPoint;
        }
    }
    // 转换为屏幕坐标，避免三维上出现显示问题
    v->SetWorldPoint(closestWAnchor.GetData());
    v->WorldToDisplay();
    double d3[3];
    v->GetDisplayPoint(d3);

    this->m_pLine->setLine(d3, edgeDPoint.GetData());
    // 更新顶点位置（使用closestWAnchor作为顶点坐标）
    if (m_bVertexVisible) {
        vtkCamera* camera = this->Renderer->GetActiveCamera();
        if (camera) {
            // 同步法线方向为相机视图方向
            double* viewDir = camera->GetViewPlaneNormal();
            m_pVertexSource->SetNormal(viewDir[0], viewDir[1], viewDir[2]);
        }
        m_pVertexSource->SetCenter(closestWAnchor.GetX(),
                                   closestWAnchor.GetY(),
                                   closestWAnchor.GetZ());
        m_pVertexSource->Modified();
    }    
}

vtkVector3d CaptionRepresentation::FindClosestPoint(const double* anchorPoint, const double* lfPoint, const double* urPoint) {
    double d2, minD2, pt[3], minPt[3];
    minD2 = VTK_DOUBLE_MAX;

    minPt[0] = lfPoint[0];
    minPt[1] = lfPoint[1];

    pt[0] = lfPoint[0];
    pt[1] = lfPoint[1];
    pt[2] = minPt[2] = 0.0;

    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[0] = (lfPoint[0] + urPoint[0]) / 2.0;
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[0] = urPoint[0];
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[1] = (lfPoint[1] + urPoint[1]) / 2.0;
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[1] = urPoint[1];
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[0] = (lfPoint[0] + urPoint[0]) / 2.0;
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[0] = lfPoint[0];
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }

    pt[1] = (lfPoint[1] + urPoint[1]) / 2.0;
    if ((d2 = vtkMath::Distance2BetweenPoints(anchorPoint, pt)) < minD2) {
        minD2 = d2;
        minPt[0] = pt[0];
        minPt[1] = pt[1];
    }
    return vtkVector3d(minPt);
}

vtkProperty2D* CaptionRepresentation::GetLineProperty() {
    return m_pLineActor->GetProperty();
}

void CaptionRepresentation::SetAnchorPoints(const std::vector<vtkVector3d>& points) {
    m_vecAnchors = points;
}

void CaptionRepresentation::BuildRepresentation() {
    this->Superclass::BuildRepresentation();
    SyncShadowTextActors();
    if (this->Renderer) {
        SyncShadowTextPositions(this->Renderer);
        // 更新顶点半径（确保像素大小不变）
        if (m_bVertexVisible && m_pVertexSource) {
            double worldRadius = CalculateVertexWorldRadius(this->Renderer);
            m_pVertexSource->SetRadius(worldRadius);
            m_pVertexSource->Modified();
        }
    }
    UpdateLine(this->Renderer);
}

void CaptionRepresentation::ReleaseGraphicsResources(vtkWindow* w) {
    this->m_pLineActor->ReleaseGraphicsResources(w);
    this->m_pVertexActor->ReleaseGraphicsResources(w);
    for (int i = 0; i < 8; ++i) {
        m_borderActors[i]->ReleaseGraphicsResources(w);
    }
    this->Superclass::ReleaseGraphicsResources(w);
}

int CaptionRepresentation::RenderOverlay(vtkViewport* v) {
    SyncShadowTextPositions(v);

    int count = 0;
    // 先渲染8个方向的描边文本
    for (int i = 0; i < 8; ++i) {
        count += m_borderActors[i]->RenderOverlay(v);
    }
    // 再渲染顶层文本
    count += this->Superclass::RenderOverlay(v);
    if (this->m_pLineActor->GetVisibility()) {
        count += this->m_pLineActor->RenderOverlay(v);
    }
    if (this->m_pVertexActor->GetVisibility()) {
        count += this->m_pVertexActor->RenderOverlay(v);
    }
    return count;
}

int CaptionRepresentation::RenderOpaqueGeometry(vtkViewport* v) {
    SyncShadowTextPositions(v);

    int count = 0;
    for (int i = 0; i < 8; ++i) {
        count += m_borderActors[i]->RenderOpaqueGeometry(v);
    }
    count += this->Superclass::RenderOpaqueGeometry(v);
    if (this->m_pLineActor->GetVisibility()) {
        count += this->m_pLineActor->RenderOpaqueGeometry(v);
    }
    if (this->m_pVertexActor->GetVisibility()) {
        count += this->m_pVertexActor->RenderOpaqueGeometry(v);
    }
    return count;
}

int CaptionRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* v) {
    SyncShadowTextPositions(v);

    int count = 0;
    for (int i = 0; i < 8; ++i) {
        count += m_borderActors[i]->RenderTranslucentPolygonalGeometry(v);
    }
    count += this->Superclass::RenderTranslucentPolygonalGeometry(v);
    if (this->m_pLineActor->GetVisibility()) {
        count += this->m_pLineActor->RenderTranslucentPolygonalGeometry(v);
    }
    if (this->m_pVertexActor->GetVisibility()) {
        count += this->m_pVertexActor->RenderTranslucentPolygonalGeometry(v);
    }
    return count;
}

vtkTypeBool CaptionRepresentation::HasTranslucentPolygonalGeometry() {
    int result = this->Superclass::HasTranslucentPolygonalGeometry();
    result |= this->m_pLineActor->HasTranslucentPolygonalGeometry();
    for (int i = 0; i < 8; ++i) {
        result |= m_borderActors[i]->HasTranslucentPolygonalGeometry();
    }
    if (this->m_pVertexActor->GetVisibility()) {
        result |= this->m_pVertexActor->HasTranslucentPolygonalGeometry();
    }
    return result;
}

vtkCoordinate* CaptionRepresentation::GetAttachmentCoordinate() {
    return m_pAttachmentCoordinate;
}

void CaptionRepresentation::SetLineVisible(bool bShow) {
    if (m_pLineActor){
        m_pLineActor->SetVisibility(bShow);
        this->Modified();
    }
}

void CaptionRepresentation::SetVertexVisible(bool show) {
    if (m_bVertexVisible != show) {
        m_bVertexVisible = show;
        m_pVertexActor->SetVisibility(show);
        this->Modified();
    }
}

bool CaptionRepresentation::GetVertexVisible() const {
    return m_bVertexVisible;
}

void CaptionRepresentation::UpdatePosition(const vtkVector3d& vertexPos, const vtkVector3d& textPos) {
    // 1. 更新顶点位置和半径
    if (m_pVertexSource && this->Renderer) {
        double worldRadius = CalculateVertexWorldRadius(this->Renderer);
        m_pVertexSource->SetRadius(worldRadius);  // 设置半径（固定像素大小）
        // 设置中心点（直接支持三维坐标）
        m_pVertexSource->SetCenter(vertexPos.GetX(),
                                   vertexPos.GetY(),
                                   vertexPos.GetZ());
        vtkCamera* camera = this->Renderer->GetActiveCamera();
        if (camera) {
            // 同步法线方向为相机视图方向
            double* viewDir = camera->GetViewPlaneNormal();
            m_pVertexSource->SetNormal(viewDir[0], viewDir[1], viewDir[2]);
        }
        m_pVertexSource->Modified();  // 触发更新
    }

    // 2. 更新文本框位置（将3D世界坐标转换为视口归一化坐标）
    if (this->Renderer) {         // 确保渲染器存在
        double displayPos[3];     // 屏幕坐标（x, y, z）
        double normalizedPos[2];  // 归一化视口坐标（x, y）

        // 步骤1：3D世界坐标 -> 屏幕坐标
        this->Renderer->SetWorldPoint(textPos[0], textPos[1], textPos[2], 1.0);  // 设置世界坐标
        this->Renderer->WorldToDisplay();                                        // 转换为屏幕坐标
        this->Renderer->GetDisplayPoint(displayPos);                             // 获取屏幕坐标（x, y, z）

        // 步骤2：屏幕坐标 -> 归一化视口坐标（范围0~1）
        // 注意：vtkRenderer的DisplayToNormalizedDisplay直接修改输入的坐标值
        normalizedPos[0] = displayPos[0];
        normalizedPos[1] = displayPos[1];
        this->Renderer->DisplayToNormalizedDisplay(normalizedPos[0], normalizedPos[1]);  // 转换为归一化坐标

        // 步骤3：设置文本框位置（vtkTextRepresentation使用归一化视口坐标）
        this->PositionCoordinate->SetValue(normalizedPos[0], normalizedPos[1]);
        this->PositionCoordinate->Modified();  // 触发文本位置更新
    }

    // 3. 更新锚点列表（确保连接线起点为新顶点位置）
    m_vecAnchors.clear();
    m_vecAnchors.push_back(vertexPos);  // 锚点关联顶点

    // 4. 重建连接线和整体表示
    this->BuildRepresentation();  // 触发UpdateLine重新计算连接线

    // 5. 通知渲染系统更新
    this->Modified();
}

vtkProperty* CaptionRepresentation::GetVertexProperty() {
    return m_pVertexActor->GetProperty();
}

void CaptionRepresentation::SetText(const char* text) {
    Superclass::SetText(text);
    if (m_bShowShadow) {
        SyncShadowTextActors();
        if (this->Renderer) {
            SyncShadowTextPositions(this->Renderer);
        }
        this->Modified();
    }
}

double CaptionRepresentation::CalculateVertexWorldRadius(vtkRenderer* renderer) {
    double pixelToWorld = boea::VtkCommon::GetWorldLengthPerPixel(renderer, m_pVertexSource->GetCenter());
    return (m_vertexFixedPixelSize * pixelToWorld) / 2.0;
}

void CaptionRepresentation::SetVertexFixedPixelSize(int size) {
    if (size > 0 && m_vertexFixedPixelSize != size) {
        m_vertexFixedPixelSize = size;
        if (this->Renderer && m_bVertexVisible) {
            double worldRadius = CalculateVertexWorldRadius(this->Renderer);
            m_pVertexSource->SetRadius(worldRadius);
            m_pVertexSource->Modified();
        }
        this->Modified();
    }
}

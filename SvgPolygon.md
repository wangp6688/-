我需要你帮我完成一个完整的 C++/VTK 几何类开发任务，这个任务分为两个阶段，但最终输出是一个“无需运行时解析 SVG”的 vtkSource 类。

【总体目标】

实现一个类似 vtkRegularPolygonSource 的类，但生成的形状不是规则多边形，而是来自一个 SVG 文件中的轮廓。

最终效果是：用户只需要设置 Center、Normal、Direction，就可以生成该 SVG 形状对应的 vtkPolyData。

--------------------------------------------------

【第一部分：SVG 内容解析（在代码生成阶段完成，不在运行时执行）】

我会提供一个 SVG 文件内容（包含 path 或 polygon 等）。

你的任务是：

1. 解析 SVG：
   - 支持 <path>（M, L, C, Q, Z 等）
   - 或 polygon / polyline
2. 将曲线（Bezier 等）离散化为折线点
3. 得到一个“闭合的二维轮廓点集”
4. 对轮廓做预处理：
   - 平移到以 (0,0) 为中心（几何中心或包围盒中心）
   - 归一化尺寸（例如最大边为 1 或保持原比例）
5. 最终生成一个二维点数组，例如：
   std::vector<std::array<double, 2>>

⚠️ 注意：
- 这一步的结果必须“固化”为 C++ 代码中的常量数据
- 不允许在最终类中出现 SVG 解析逻辑

--------------------------------------------------

【第二部分：VTK 几何类实现】

基于上一步得到的二维轮廓，实现一个 VTK 类：

类名：vtkSVGHandleSource  
继承：vtkPolyDataAlgorithm  

--------------------------------------------------

【类功能要求】

1. 输入参数：

- Center(double[3])
- Normal(double[3])
- Direction(double[3])
- Scale(double)

2. 内部数据：

- 一个“写死”的二维轮廓点数组（来自 SVG 解析结果）

3. 几何生成逻辑：

- 构建局部坐标系：
  Z = normalize(Normal)
  X = normalize(Direction - dot(Direction, Z) * Z)
  Y = cross(Z, X)

- 将二维点 (x, y) 映射到三维：
  P = Center + Scale * (x * X + y * Y)

4. 输出：

- vtkPolyData
- 包含：
  - vtkPoints
  - vtkCellArray（闭合 polygon）

--------------------------------------------------

【实现要求】

- 完整 .h 和 .cxx
- 符合 VTK 9.x 规范
- 实现 RequestData()
- 提供 Set/Get 接口：
  - SetCenter
  - SetNormal
  - SetDirection
  - SetScale

- 数值稳定性处理：
  - Direction 与 Normal 不正交时自动修正
  - 防止零向量
  - 保证法向一致性（右手系）

--------------------------------------------------

【附加要求（重要）】

- 不要依赖任何 SVG 解析库（如 nanoSVG、tinyxml 等）
- SVG 的解析结果必须已经转化为“静态点数据”
- 输出代码应可直接编译运行
- 风格参考 vtkRegularPolygonSource

--------------------------------------------------

【输入】

接下来我会提供 SVG 文件内容，请你：
1. 解析它
2. 生成轮廓点
3. 输出完整的 vtkSVGHandleSource 实现代码

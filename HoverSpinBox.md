# Qt 自定义角度输入框需求

请帮我实现一个继承自 `QDoubleSpinBox` 的控件，类名为 `HoverSpinBox`。

## 功能概述

该控件用于显示和编辑角度值，支持两种工作模式：

```cpp
enum class AngleType
{
    Horizontal, // 水平方向角度
    Vertical    // 垂直方向角度
};
```

* Horizontal：角度范围为 [-180°, 180°]
* Vertical：角度范围为 [-90°, 90°]

控件内部保存真实角度值（带正负号）。

---

## 基础要求

### 小数位数

该控件只允许整数角度：

```cpp
setDecimals(0);
```

例如：

```text
10°
35°
180°
```

不允许：

```text
10.5°
35.2°
```

---

## 显示规则

### 显示角度后缀

显示文本时自动追加：

```text
°
```

例如：

```text
45°
120°
```

---

### 只显示绝对值

显示时不显示正负号，只显示绝对值。

例如：

| 实际值  | 显示文本 |
| ---- | ---- |
| 30   | 30°  |
| -30  | 30°  |
| 180  | 180° |
| -180 | 180° |

即：

```cpp
displayText = QString::number(std::abs(value)) + "°";
```

---

## Horizontal 模式

### 范围

```cpp
[-180, 180]
```

### 点击上箭头

```text
179 -> 180
180 -> -180
-180 -> -179
-179 -> -178
```

即达到 180 后继续增加时循环到 -180。

---

### 点击下箭头

```text
-179 -> -180
-180 -> 180
180 -> 179
179 -> 178
```

即达到 -180 后继续减少时循环到 180。

---

### 本质要求

Horizontal 模式采用循环角度逻辑：

```text
[-180,180]
```

形成闭环。

---

## Vertical 模式

### 范围

```cpp
[-90, 90]
```

### 点击上箭头

```text
89 -> 90
90 -> 90
```

达到上限后保持不变。

---

### 点击下箭头

```text
-89 -> -90
-90 -> -90
```

达到下限后保持不变。

---

### 本质要求

Vertical 模式采用钳制逻辑：

```cpp
value = std::clamp(value, -90.0, 90.0);
```

---

## 用户输入规则

由于界面只显示绝对值，因此需要保留一个“当前符号状态”的概念。

### 当前值为负数

假设内部值：

```cpp
value = -30;
```

显示：

```text
30°
```

#### 用户输入正数

输入：

```text
50
```

结果：

```cpp
value = -50;
```

显示：

```text
50°
```

即：

> 输入正数时，保持当前符号不变。

---

#### 用户输入负数

输入：

```text
-50
```

结果：

```cpp
value = 50;
```

显示：

```text
50°
```

即：

> 输入负数时，翻转当前符号。

---

### 当前值为正数

假设内部值：

```cpp
value = 30;
```

显示：

```text
30°
```

#### 用户输入正数

输入：

```text
50
```

结果：

```cpp
value = 50;
```

显示：

```text
50°
```

保持当前符号。

---

#### 用户输入负数

输入：

```text
-50
```

结果：

```cpp
value = -50;
```

显示：

```text
50°
```

翻转当前符号。

---

## 输入规则总结

设：

```cpp
currentSign =
    (currentValue >= 0) ? 1 : -1;
```

如果输入的是正数：

```cpp
newValue =
    currentSign * abs(inputValue);
```

保持当前符号。

如果输入的是负数：

```cpp
newValue =
    -currentSign * abs(inputValue);
```

翻转当前符号。

因此：

| 当前值 | 用户输入 | 最终值 |
| --- | ---- | --- |
| 30  | 50   | 50  |
| 30  | -50  | -50 |
| -30 | 50   | -50 |
| -30 | -50  | 50  |

---

## 其它要求

* 支持键盘输入。
* 支持上下箭头按钮。
* 支持鼠标滚轮调整。
* 输入框显示内容始终带 `°`。
* 显示内容始终为绝对值。
* 内部保存真实带符号角度值。
* Horizontal 模式采用循环角度。
* Vertical 模式采用钳制角度。
* 小数位数固定为 0。

---

## 建议实现方式

请通过继承 `QDoubleSpinBox` 实现。

建议重写：

```cpp
QString textFromValue(double value) const override;
double valueFromText(const QString& text) const override;
QValidator::State validate(QString& text, int& pos) const override;
void stepBy(int steps) override;
```

如有必要也可以重写：

```cpp
void fixup(QString& text) const override;
```

请提供完整头文件和源文件实现，并添加必要注释说明关键逻辑。

### 范围处理规则

对于用户手动输入的数值，当输入值超出当前模式允许的范围时，行为应完全遵循 `QDoubleSpinBox` 的默认实现，不要额外增加自定义逻辑。

例如：

* Horizontal 模式范围为 `[-180, 180]`
* Vertical 模式范围为 `[-90, 90]`

对于：

```text
200
-200
100
-100
```

等超出范围的输入，交由 `QDoubleSpinBox` 自身的校验、修正和取值机制处理。

### 注意

只有点击上下箭头、鼠标滚轮等步进操作时，才需要应用本控件的特殊逻辑：

* Horizontal：循环（Wrap）逻辑
* Vertical：钳制（Clamp）逻辑

对于键盘输入解析过程，不要实现额外的循环逻辑，不要将：

```text
181 -> -180
182 -> -179
```

之类的规则应用到用户输入场景。

应尽量复用 `QDoubleSpinBox` 默认行为，仅实现：

1. 显示绝对值；
2. 显示 `°` 后缀；
3. 特殊符号切换输入规则；
4. Horizontal 步进循环；
5. Vertical 步进钳制；
6. 整数角度（`decimals = 0`）。

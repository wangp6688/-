#pragma once

#include <QDoubleSpinBox>

class QEvent;

/**
 * @brief HoverSpinBox — 角度输入框
 *
 * 继承自 QDoubleSpinBox，用于显示和编辑整数角度值。
 *
 * 功能特性：
 *  - 固定小数位数为 0（只允许整数角度）
 *  - 显示文本始终为绝对值 + "°" 后缀（如 "30°"）
 *  - 内部保存真实带符号角度值
 *  - Horizontal 模式：范围 [-180, 180]，步进时循环（Wrap）
 *  - Vertical 模式：范围 [-90, 90]，步进时钳制（Clamp）
 *  - 用户输入时保持/翻转符号：输入正数保持当前符号，输入负数翻转当前符号
 */
class HoverSpinBox : public QDoubleSpinBox
{
    Q_OBJECT

public:
    /**
     * @brief 角度类型枚举
     */
    enum class AngleType
    {
        Horizontal, ///< 水平方向角度，范围 [-180, 180]，步进时循环
        Vertical    ///< 垂直方向角度，范围 [-90, 90]，步进时钳制
    };

    /**
     * @brief 构造函数
     * @param type   角度类型（默认 Horizontal）
     * @param parent 父窗口部件
     */
    explicit HoverSpinBox(AngleType type = AngleType::Horizontal,
                          QWidget *parent = nullptr);

    /** @brief 设置角度类型，同时更新内部范围 */
    void setAngleType(AngleType type);

    /** @brief 获取当前角度类型 */
    AngleType angleType() const { return m_angleType; }

protected:
    /** 鼠标进入时显示上下箭头按钮（禁用时不处理） */
    void enterEvent(QEvent *event) override;

    /** 鼠标离开时隐藏上下箭头按钮（禁用时不处理） */
    void leaveEvent(QEvent *event) override;

    /**
     * @brief 将内部值转为显示文本
     *
     * 显示绝对值 + "°"，例如 -30 -> "30°"
     */
    QString textFromValue(double value) const override;

    /**
     * @brief 将显示文本解析为内部值
     *
     * 应用符号继承规则：
     *  - 用户输入正数 -> 保持当前符号
     *  - 用户输入负数 -> 翻转当前符号
     */
    double valueFromText(const QString &text) const override;

    /**
     * @brief 验证输入文本的合法性
     *
     * 接受格式：[-]digits[°]
     * 超出范围的数值返回 Intermediate，由 QDoubleSpinBox 默认机制处理。
     */
    QValidator::State validate(QString &text, int &pos) const override;

    /**
     * @brief 步进操作（上下箭头、鼠标滚轮）
     *
     * - Horizontal：超出 ±180 时循环
     * - Vertical：超出 ±90 时钳制
     */
    void stepBy(int steps) override;

private:
    AngleType m_angleType;

    /** 根据 m_angleType 初始化范围和单步值 */
    void applyRangeForType();
};

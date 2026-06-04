#include "HoverSpinBox.h"

#include <QLineEdit>
#include <QRegularExpression>
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// 构造 / 配置
// ─────────────────────────────────────────────────────────────────────────────

HoverSpinBox::HoverSpinBox(QWidget *parent)
    : HoverSpinBox(AngleType::Horizontal, parent)
{
}

HoverSpinBox::HoverSpinBox(AngleType type, QWidget *parent)
    : QDoubleSpinBox(parent)
    , m_angleType(type)
{
    // 只允许整数角度
    setDecimals(0);
    setSingleStep(1.0);
    setSuffix(QStringLiteral("°"));
    setButtonSymbols(QAbstractSpinBox::NoButtons);

    applyRangeForType();
}

void HoverSpinBox::setAngleType(AngleType type)
{
    if (m_angleType == type)
        return;

    m_angleType = type;
    applyRangeForType();
}

void HoverSpinBox::applyRangeForType()
{
    if (m_angleType == AngleType::Horizontal)
        setRange(-180.0, 180.0);
    else
        setRange(-90.0, 90.0);
}

void HoverSpinBox::enterEvent(QEvent *event)
{
    if (!isEnabled())
        return;

    setButtonSymbols(QAbstractSpinBox::UpDownArrows);
    QDoubleSpinBox::enterEvent(event);
}

void HoverSpinBox::leaveEvent(QEvent *event)
{
    if (!isEnabled())
        return;

    setButtonSymbols(QAbstractSpinBox::NoButtons);
    QDoubleSpinBox::leaveEvent(event);
}

// ─────────────────────────────────────────────────────────────────────────────
// 显示：返回绝对值数字，"°" 由 suffix 追加
// ─────────────────────────────────────────────────────────────────────────────

QString HoverSpinBox::textFromValue(double value) const
{
    return QString::number(static_cast<int>(std::abs(value)));
}

// ─────────────────────────────────────────────────────────────────────────────
// 解析用户输入，应用符号继承规则
// ─────────────────────────────────────────────────────────────────────────────

double HoverSpinBox::valueFromText(const QString &text) const
{
    // 去除后缀及首尾空白
    QString t = text.trimmed();
    const QString sfx = suffix();
    if (!sfx.isEmpty() && t.endsWith(sfx))
        t.chop(sfx.size());
    t = t.trimmed();

    bool ok = false;
    double inputValue = t.toDouble(&ok);
    if (!ok)
        return value(); // 解析失败时保持当前值

    // ── 符号继承规则 ──────────────────────────────────────────────────────────
    // currentSign = 当前内部值的符号（0 视为正）
    double currentValue = value();
    int currentSign = (currentValue >= 0.0) ? 1 : -1;

    double absInput = std::abs(inputValue);
    double newValue;

    if (inputValue >= 0.0)
        // 用户输入正数：保持当前符号
        newValue = currentSign * absInput;
    else
        // 用户输入负数：翻转当前符号
        newValue = -currentSign * absInput;

    return newValue;
}

// ─────────────────────────────────────────────────────────────────────────────
// 验证输入
// ─────────────────────────────────────────────────────────────────────────────

QValidator::State HoverSpinBox::validate(QString &text, int &pos) const
{
    Q_UNUSED(pos)

    // 去除后缀
    QString t = text.trimmed();
    const QString sfx = suffix();
    if (!sfx.isEmpty() && t.endsWith(sfx))
        t.chop(sfx.size());
    t = t.trimmed();

    // 空串或仅有负号视为中间状态（用户还在输入）
    if (t.isEmpty() || t == QLatin1String("-"))
        return QValidator::Intermediate;

    // 只接受 [-]digits 格式（整数）
    static const QRegularExpression re(QStringLiteral("^-?\\d{1,5}$"));
    if (!re.match(t).hasMatch())
        return QValidator::Invalid;

    // 数值超出范围时，保持 Intermediate，让 QDoubleSpinBox 默认机制处理修正
    bool ok = false;
    double val = t.toDouble(&ok);
    if (!ok)
        return QValidator::Invalid;

    const int currentSign = (value() >= 0.0) ? 1 : -1;
    const double absInput = std::abs(val);
    const double interpreted =
        (val >= 0.0) ? (currentSign * absInput) : (-currentSign * absInput);

    if (interpreted >= minimum() && interpreted <= maximum())
        return QValidator::Acceptable;

    // 格式合法但超出范围 -> Invalid（禁止输入越界值）
    return QValidator::Invalid;
}

// ─────────────────────────────────────────────────────────────────────────────
// 步进：箭头按钮 / 鼠标滚轮
// ─────────────────────────────────────────────────────────────────────────────

void HoverSpinBox::stepBy(int steps)
{
    int currentValue = static_cast<int>(value());

    if (m_angleType == AngleType::Horizontal)
    {
        // ── 循环（Wrap）逻辑 ─────────────────────────────────────────────────
        // 范围 [-180, 180]，共 361 个整数值，形成闭环。
        // 使用带正余数的模运算保证结果始终落在 [-180, 180]。
        int newValue = currentValue + steps;
        const int period = 361; // 180 - (-180) + 1
        const int offset = 180;
        // 将区间 [-180, 180] 映射到 [0, 360]，取模后再映射回来
        newValue = ((newValue + offset) % period + period) % period - offset;
        setValue(static_cast<double>(newValue));
    }
    else
    {
        // ── 钳制（Clamp）逻辑 ────────────────────────────────────────────────
        int newValue = currentValue + steps;
        newValue = std::max(-90, std::min(90, newValue));
        setValue(static_cast<double>(newValue));
    }
}

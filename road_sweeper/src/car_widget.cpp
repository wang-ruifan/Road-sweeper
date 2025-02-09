#include "car_widget.hpp"
#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include <QResizeEvent>

CarWidget::CarWidget(QWidget *parent)
    : QWidget(parent)
    , m_leftLineAngle(0.0)
    , m_rightLineAngle(0.0)
    , m_lineLength(50)
{
    setMinimumSize(200, 200);
}

void CarWidget::setCarImage(const QString& imagePath)
{
    if (m_originalImage.load(imagePath)) {
        updateScaledImage();
        update();
    } else {
        qDebug() << "Failed to load car image:" << imagePath;
    }
}

void CarWidget::setLinesBending(qreal leftAngle, qreal rightAngle)
{
    m_leftLineAngle = leftAngle;
    m_rightLineAngle = rightAngle;
    update();
}

void CarWidget::setAngle(qreal angle)
{
    m_leftLineAngle = angle;
    m_rightLineAngle = angle;
    update();
}

void CarWidget::setLineLength(int length)
{
    m_lineLength = length;
    update();
}

void CarWidget::updateScaledImage()
{
    if (!m_originalImage.isNull()) {
        // 首先将图片缩放到窗口大小的3/4
        m_scaledImage = m_originalImage.scaled(
            width() * 3 / 4, height() * 3 / 4, 
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        );
    }
}

void CarWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    updateScaledImage();
}

void CarWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制小车图片
    if (!m_scaledImage.isNull()) {
        // 计算位置：水平居中，垂直靠底部
        int x = (width() - m_scaledImage.width()) / 2;
        int y = height() - m_scaledImage.height();
        painter.drawPixmap(x, y, m_scaledImage);

        // 计算线条起始点
        int leftLineX = x + m_scaledImage.width() / 4;
        int rightLineX = x + (m_scaledImage.width() * 3) / 4;
        int lineY = y + m_scaledImage.height() / 5;

        // 设置线条长度为起点到控件顶部的距离
        m_lineLength = lineY;  // 从起点到顶部的距离

        // 设置线的样式
        QPen pen(Qt::red);
        pen.setWidth(2);
        painter.setPen(pen);

        const qreal verticalOffset = 0.7;

        // 绘制左侧曲线
        {
            QPainterPath path;
            path.moveTo(leftLineX, lineY);
            
            // 计算终点（y坐标改为0，表示控件顶部）
            QPoint endPoint(
                leftLineX + m_lineLength * qSin(qDegreesToRadians(m_leftLineAngle)),
                0  // 终点设为控件顶部
            );
            
            QPoint controlPoint(
                leftLineX + (endPoint.x() - leftLineX) * 0.5,
                lineY - m_lineLength * verticalOffset
            );
            
            path.quadTo(controlPoint, endPoint);
            painter.drawPath(path);
        }

        // 绘制右侧曲线
        {
            QPainterPath path;
            path.moveTo(rightLineX, lineY);
            
            QPoint endPoint(
                rightLineX + m_lineLength * qSin(qDegreesToRadians(m_rightLineAngle)),
                0  // 终点设为控件顶部
            );
            
            QPoint controlPoint(
                rightLineX + (endPoint.x() - rightLineX) * 0.5,
                lineY - m_lineLength * verticalOffset
            );
            
            path.quadTo(controlPoint, endPoint);
            painter.drawPath(path);
        }
    }
}
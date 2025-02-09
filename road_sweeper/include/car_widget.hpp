#ifndef CARWIDGET_H
#define CARWIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QPainterPath>
#include <QtMath>

class CarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CarWidget(QWidget *parent = nullptr);
    void setCarImage(const QString& imagePath);
    void setLinesBending(qreal leftAngle, qreal rightAngle);
    void setAngle(qreal angle);
    void setLineLength(int length);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    void updateScaledImage();

private:
    QPixmap m_originalImage;
    QPixmap m_scaledImage;
    qreal m_leftLineAngle;
    qreal m_rightLineAngle;
    int m_lineLength;
};

#endif // CARWIDGET_H
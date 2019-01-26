#pragma once

#include <QtWidgets>

class ImageLabel: public QLabel
{
    Q_OBJECT
public:
    ImageLabel(const int &w = 640, const int &h = 480);
    void set_image(QImage im);
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

signals:
    void shot(QRect);
private:
    bool drawing;
    QPoint startPoint;
    QPoint endPoint;
    QRect shotRect;
    QImage image;
};

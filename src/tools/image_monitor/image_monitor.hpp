#pragma once

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client.hpp"
#include "ui/ImageLabel.hpp"
#include <opencv2/opencv.hpp>

class image_monitor: public QMainWindow
{
    Q_OBJECT
public:
    image_monitor();
    void data_handler(const tcp_command cmd);
public slots:
    void procTimer();
    void procYawSlider(int v);
    void procPitchSlider(int v);
    void procBtnWR();
    void procBtnCS();
    void procShot(QRect rect);
    void procImageBox(int idx);

protected:
    void closeEvent(QCloseEvent *event);
private:
    QPushButton *btnWR, *btnCS;
    ImageLabel *imageLab;
    QLabel *yawLab, *pitchLab, *netLab;
    QSlider *pitchSlider, *yawSlider;
    QComboBox *colorBox;
    QCheckBox *colorCheck;
    QComboBox *imageBox;
    QTimer *timer;
    tcp_client client_;
    QString net_info;
    bool first_connect;
    cv::Mat curr_image_;
    int width_;
    int height_;
};

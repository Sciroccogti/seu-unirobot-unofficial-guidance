#pragma once

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client.hpp"

class JSlider: public QWidget
{
    Q_OBJECT
public:
    JSlider(robot::joint_ptr j)
        : name_(j->name_), id_(j->jid_), range_(100), scale_(10.0)
    {
        nameLab = new QLabel(QString::fromStdString(name_));
        nameLab->setFixedWidth(100);
        slider = new QSlider(Qt::Horizontal);
        slider->setMinimumWidth(200);
        slider->setMaximum(range_);
        slider->setMinimum(-range_);
        slider->setValue(static_cast<int>(scale_ * j->offset_));
        dataLab = new QLabel(QString::number(j->offset_, 'f', 1));
        dataLab->setFixedWidth(40);
        QHBoxLayout *mainLayout = new QHBoxLayout;
        mainLayout->addWidget(nameLab);
        mainLayout->addWidget(slider);
        mainLayout->addWidget(dataLab);
        setLayout(mainLayout);
        connect(slider, &QSlider::valueChanged, this, &JSlider::procSliderChanged);
    }

    void reset()
    {
        slider->setValue(0);
        procSliderChanged(0);
    }
public slots:
    void procSliderChanged(int v)
    {
        float offset = v / scale_;
        dataLab->setText(QString::number(offset, 'f', 1));
        emit valueChanged(id_, offset);
    }
signals:
    void valueChanged(int id, float v);

private:
    QSlider *slider;
    QLabel *nameLab, *dataLab;
    std::string name_;
    int id_;
    int range_;
    float scale_;
};

class joint_revise: public QMainWindow
{
    Q_OBJECT
public:
    joint_revise(tcp_client &client, QString netinfo, QWidget *parent = nullptr);
public slots:
    void procBtnReset();
    void procBtnSave();
    void procValueChanged(int id, float v);
    void procTimer();

private:
    std::map<std::string, JSlider *> j_sliders_;
    QPushButton *btnReset, *btnSave;
    QTimer *timer;
    tcp_client &client_;
    QString net_info;
    bool first_connect;
};

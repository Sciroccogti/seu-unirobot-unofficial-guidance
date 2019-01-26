#pragma once

#include <QtWidgets>
#include "model.hpp"
#include "tcp_client/tcp_client.hpp"

#define CAMARA_PARA_SCALE 100.0f

class CtrlSlider: public QWidget
{
    Q_OBJECT
public:
    CtrlSlider(const camera_info &info)
        : info_(info)
    {
        qRegisterMetaType<camera_info>("para_info");
        nameLab = new QLabel(QString::fromStdString(info.name));
        slider = new QSlider(Qt::Horizontal);
        slider->setMinimumWidth(200);
        slider->setMaximum(info.max_value * CAMARA_PARA_SCALE);
        slider->setMinimum(info.min_value * CAMARA_PARA_SCALE);
        slider->setValue(info.value * CAMARA_PARA_SCALE);
        dataLab = new QLabel(QString::number(info.value));
        dataLab->setFixedWidth(40);
        QHBoxLayout *sLayout = new QHBoxLayout;
        sLayout->addWidget(slider);
        sLayout->addWidget(dataLab);
        QVBoxLayout *mainLayout = new QVBoxLayout;
        mainLayout->addWidget(nameLab);
        mainLayout->addLayout(sLayout);
        setLayout(mainLayout);
        connect(slider, &QSlider::valueChanged, this, &CtrlSlider::procSliderChanged);
    }

    void reset()
    {
        slider->setValue(info_.default_value);
        procSliderChanged(info_.default_value);
    }

    camera_info c_info() const
    {
        return info_;
    }
public slots:
    void procSliderChanged(int v)
    {
        dataLab->setText(QString::number(v / CAMARA_PARA_SCALE));
        info_.value = v / CAMARA_PARA_SCALE;
        emit valueChanged(info_);
    }
signals:
    void valueChanged(camera_info info);

private:
    camera_info info_;
    QSlider *slider;
    QLabel *nameLab, *dataLab;
};

class camera_setter: public QMainWindow
{
    Q_OBJECT
public:
    camera_setter(tcp_client &client, QString netinfo, QWidget *parent = nullptr);
public slots:
    void procBtnReset();
    void procBtnSave();
    void procValueChanged(camera_info info);
    void procTimer();

private:
    std::vector<CtrlSlider *> c_sliders_;
    std::map<std::string, camera_info> ctrl_items_;
    QPushButton *btnReset, *btnSave;
    QTimer *timer;
    tcp_client &client_;
    QString net_info;
    bool first_connect;
};

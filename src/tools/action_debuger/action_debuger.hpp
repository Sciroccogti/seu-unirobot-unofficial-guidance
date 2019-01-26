#pragma once

#include <QtWidgets>
#include <map>
#include "ui/RobotGL.hpp"
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client.hpp"

#define SLIDER_RANGE 1000

class CPosListWidget : public QWidget
{
    Q_OBJECT
public:
    QLabel *m_id;
    QLineEdit *pos_name;
    QLineEdit *pos_time;
    std::string pos_name_;
    int time_;
public:
    CPosListWidget(const int &id, const std::string &p_name, const int &t)
        : pos_name_(p_name), time_(t)
    {
        m_id = new QLabel(QString::number(id));
        pos_name = new QLineEdit(QString::fromStdString(p_name));
        pos_time = new QLineEdit(QString::number(t));
        pos_time->setFixedWidth(40);
        QHBoxLayout *mainLayout = new QHBoxLayout;
        mainLayout->addWidget(m_id);
        mainLayout->addWidget(pos_name);
        mainLayout->addWidget(pos_time);
        setLayout(mainLayout);
        connect(pos_name, &QLineEdit::editingFinished, this, &CPosListWidget::procNameChange);
        connect(pos_time, &QLineEdit::editingFinished, this, &CPosListWidget::procTimeChange);
    }
private slots:
    void procNameChange()
    {
        emit nameChanged(m_id->text().toInt());
    }
    void procTimeChange()
    {
        int t = pos_time->text().toInt();

        if (t == 0)
        {
            QMessageBox::warning(this, "Error", "Invalid time!");
            pos_time->setText(QString::number(time_));
            return;
        }

        time_ = t;
        emit timeChanged(m_id->text().toInt());
    }
signals:
    void nameChanged(int);
    void timeChanged(int);
};

class CJointDegWidget : public QWidget
{
    Q_OBJECT
public:
    QLabel *name;
    QLabel *deg;
public:
    CJointDegWidget(const std::string &j_name, const float &j_deg)
    {
        name = new QLabel(QString::fromStdString(j_name));
        deg = new QLabel(QString::number(j_deg, 'f', 2));
        deg->setFixedWidth(60);
        QHBoxLayout *mainLayout = new QHBoxLayout;
        mainLayout->addWidget(name);
        mainLayout->addWidget(deg);
        setLayout(mainLayout);
    }
};

class CKSlider: public QWidget
{
    Q_OBJECT
public:
    QLabel *nameLab;
    QSlider *slider;
public:
    CKSlider(const std::string &key)
    {
        nameLab = new QLabel(QString::fromStdString(key));
        nameLab->setFixedWidth(100);
        slider = new QSlider(Qt::Horizontal);
        slider->setMaximum(SLIDER_RANGE);
        slider->setMinimum(-SLIDER_RANGE);
        slider->setValue(0);
        QHBoxLayout *mainLayout = new QHBoxLayout;
        mainLayout->addWidget(nameLab);
        mainLayout->addWidget(slider);
        setLayout(mainLayout);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procValueChanged(int)));
    }
public slots:
    void procValueChanged(int value)
    {
        emit valueChanged(value);
    };
signals:
    void valueChanged(int value);
};

class action_debuger: public QMainWindow
{
    Q_OBJECT
public:
    action_debuger();
    void initStatusBar();
protected:
    void closeEvent(QCloseEvent *event);
private slots:
    void procX(int value);
    void procY(int value);
    void procZ(int value);
    void procRoll(int value);
    void procPitch(int value);
    void procYaw(int value);

    void procActSelect(QListWidgetItem *item);
    void procPosSelect(QListWidgetItem *item);

    void procButtonInsertPosBack();
    void procButtonInsertPosFront();
    void procButtonDeletePos();
    void procButtonSavePos();
    void procButtonAddAction();
    void procButtonDeleteAction();
    void procButtonSaveAction();

    void procButtonRunPos();
    void procButtonWalkRemote();
    void procButtonJointRevise();
    void updateSlider(int id);
    void procTimer();
    void procPosNameChanged(int id);
    void procPosTimeChanged(int id);

private:
    void updateJDInfo();
    void updatePosList(std::string act_name);
    void removeUnusedPos();
    void initActs();
    void initPoseMap();
    void initJDInfo();
    float get_deg_from_pose(const float &ps);
    bool turn_joint();

    robot::robot_motion motion_, last_motion;
    std::map<robot::robot_motion, robot::robot_pose> pose_map_;
    std::map<int, float> joint_degs_;
    QTimer *timer;
    tcp_client client_;
    QString net_info;
    bool first_connect;
    bool pos_saved;
    int last_pos_id, last_act_id;

    QLabel *motionlab, *valuelab, *currposlab, *curractlab, *netstatuslab;
    QListWidget *m_pPosListWidget, *m_pActListWidget, *m_pJDListWidget1, *m_pJDListWidget2;
    QPushButton *mButtonInsertPosFront, *mButtonInsertPosBack, *mButtonDeletePos, *mButtonSavePos;
    QPushButton *mButtonAddAction, *mButtonDeleteAction, *mButtonSaveAction;
    QPushButton *btnrunPos, *btnWalkRemote, *btnJointRevise;
    QRadioButton *head, *body, *leftArm, *rightArm, *leftFoot, *rightFoot;
    QButtonGroup *motionBtnGroup;
    QGroupBox *mSliderGroup;
    RobotGL *robot_gl_;
    std::vector<CKSlider *> mKsliders;
    std::map<std::string, CJointDegWidget *> mJDInfos;
};

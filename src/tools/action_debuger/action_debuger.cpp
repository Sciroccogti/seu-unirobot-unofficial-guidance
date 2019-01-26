#include <fstream>
#include "action_debuger.hpp"
#include "ui/walk_remote.hpp"
#include "ui/joint_revise.hpp"
#include "math/math.hpp"
#include "configuration.hpp"
#include "parser/action_parser.hpp"

#define SCALE_K  0.001f
#define DEG_RANGE 180.0f
#define MAX_POS_ID 100

using namespace parser;
using namespace std;
using namespace robot;
using namespace robot_math;
using namespace Eigen;

action_debuger::action_debuger()
    : client_(CONF->get_config_value<string>(CONF->player() + ".address"), CONF->get_config_value<int>("net.tcp.port"))
{
    initStatusBar();
    robot_gl_ = new RobotGL(ROBOT->get_main_bone(), ROBOT->get_joint_map());
    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addWidget(robot_gl_);

    mKsliders.clear();
    CKSlider *slider;

    slider = new CKSlider("X");
    connect(slider, &CKSlider::valueChanged, this, &action_debuger::procX);
    mKsliders.push_back(slider);
    slider = new CKSlider("Y");
    connect(slider, &CKSlider::valueChanged, this, &action_debuger::procY);
    mKsliders.push_back(slider);
    slider = new CKSlider("Z");
    connect(slider, &CKSlider::valueChanged, this, &action_debuger::procZ);
    mKsliders.push_back(slider);
    slider = new CKSlider("Roll");
    connect(slider, &CKSlider::valueChanged, this, &action_debuger::procRoll);
    mKsliders.push_back(slider);
    slider = new CKSlider("Pitch");
    connect(slider, &CKSlider::valueChanged, this, &action_debuger::procPitch);
    mKsliders.push_back(slider);
    slider = new CKSlider("Yaw");
    connect(slider, &CKSlider::valueChanged, this, &action_debuger::procYaw);
    mKsliders.push_back(slider);

    mSliderGroup = new QGroupBox();
    QVBoxLayout *sliderLayout = new QVBoxLayout();

    for (auto &s : mKsliders)
    {
        sliderLayout->addWidget(s);
    }

    mSliderGroup->setLayout(sliderLayout);

    mButtonSavePos = new QPushButton(tr("Save Pos"));
    head = new QRadioButton("Head");
    body = new QRadioButton("Body");
    leftArm = new QRadioButton("Left Arm");
    rightArm = new QRadioButton("Right Arm");
    leftFoot = new QRadioButton("Left Foot");
    rightFoot = new QRadioButton("Right Foot");

    m_pJDListWidget1 = new QListWidget;
    m_pJDListWidget2 = new QListWidget;

    motionBtnGroup = new QButtonGroup();
    motionBtnGroup->addButton(head, static_cast<int>(MOTION_HEAD));
    motionBtnGroup->addButton(body, static_cast<int>(MOTION_BODY));
    motionBtnGroup->addButton(leftArm, static_cast<int>(MOTION_LEFT_HAND));
    motionBtnGroup->addButton(rightArm, static_cast<int>(MOTION_RIGHT_HAND));
    motionBtnGroup->addButton(leftFoot, static_cast<int>(MOTION_LEFT_FOOT));
    motionBtnGroup->addButton(rightFoot, static_cast<int>(MOTION_RIGHT_FOOT));
    body->setChecked(true);

    QHBoxLayout *hbLayout = new QHBoxLayout();
    hbLayout->addWidget(head);
    hbLayout->addWidget(body);
    QHBoxLayout *armLayout = new QHBoxLayout();
    armLayout->addWidget(rightArm);
    armLayout->addWidget(leftArm);
    QHBoxLayout *footLayout = new QHBoxLayout();
    footLayout->addWidget(rightFoot);
    footLayout->addWidget(leftFoot);
    QHBoxLayout *jdLayout = new QHBoxLayout();
    jdLayout->addWidget(m_pJDListWidget1);
    jdLayout->addWidget(m_pJDListWidget2);

    QVBoxLayout *midLayout = new QVBoxLayout;
    midLayout->setAlignment(Qt::AlignCenter);
    midLayout->addWidget(mSliderGroup);
    midLayout->addLayout(hbLayout);
    midLayout->addLayout(armLayout);
    midLayout->addLayout(footLayout);
    midLayout->addLayout(jdLayout);
    midLayout->addWidget(mButtonSavePos);

    m_pPosListWidget = new QListWidget();
    m_pPosListWidget->setMinimumWidth(240);
    btnrunPos = new QPushButton("Run Pos");
    btnWalkRemote = new QPushButton("Walk Remote");
    btnJointRevise = new QPushButton("Joint Revise");
    QVBoxLayout *posLayout = new QVBoxLayout;
    posLayout->addWidget(m_pPosListWidget);
    posLayout->addWidget(btnrunPos);

    m_pActListWidget = new QListWidget();
    mButtonInsertPosFront = new QPushButton(tr("Insert Pos Front"));
    mButtonInsertPosBack = new QPushButton(tr("Insert Pos Back"));
    mButtonDeletePos = new QPushButton(tr("Delete Pos"));
    mButtonSaveAction = new QPushButton(tr("Save Actions"));
    mButtonAddAction = new QPushButton(tr("Add Action"));
    mButtonDeleteAction = new QPushButton(tr("Delete Action"));

    QVBoxLayout *bactLayout = new QVBoxLayout;
    bactLayout->addWidget(mButtonInsertPosFront);
    bactLayout->addWidget(mButtonInsertPosBack);
    bactLayout->addWidget(mButtonDeletePos);
    bactLayout->addWidget(mButtonSaveAction);
    bactLayout->addWidget(mButtonAddAction);
    bactLayout->addWidget(mButtonDeleteAction);
    bactLayout->addWidget(m_pActListWidget);
    bactLayout->addWidget(btnWalkRemote);
    bactLayout->addWidget(btnJointRevise);

    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(midLayout);
    mainLayout->addLayout(posLayout);
    mainLayout->addLayout(bactLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    pos_saved = true;
    last_pos_id = MAX_POS_ID;
    initActs();
    initPoseMap();
    initJDInfo();
    net_info = QString::fromStdString(CONF->get_config_value<string>(CONF->player() + ".address"))
               + ":" + QString::number(CONF->get_config_value<int>("net.tcp.port"));
    setWindowTitle(net_info);

    timer = new QTimer;
    timer->start(1000);

    connect(m_pActListWidget, &QListWidget::itemClicked, this, &action_debuger::procActSelect);
    connect(m_pPosListWidget, &QListWidget::itemClicked, this, &action_debuger::procPosSelect);
    connect(mButtonAddAction, &QPushButton::clicked, this, &action_debuger::procButtonAddAction);
    connect(mButtonDeleteAction, &QPushButton::clicked, this, &action_debuger::procButtonDeleteAction);
    connect(mButtonSaveAction, &QPushButton::clicked, this, &action_debuger::procButtonSaveAction);
    connect(mButtonInsertPosFront, &QPushButton::clicked, this, &action_debuger::procButtonInsertPosFront);
    connect(mButtonInsertPosBack, &QPushButton::clicked, this, &action_debuger::procButtonInsertPosBack);
    connect(mButtonDeletePos, &QPushButton::clicked, this, &action_debuger::procButtonDeletePos);
    connect(mButtonSavePos, &QPushButton::clicked, this, &action_debuger::procButtonSavePos);
    connect(btnrunPos, &QPushButton::clicked, this, &action_debuger::procButtonRunPos);
    connect(btnWalkRemote, &QPushButton::clicked, this, &action_debuger::procButtonWalkRemote);
    connect(btnJointRevise, &QPushButton::clicked, this, &action_debuger::procButtonJointRevise);
    connect(motionBtnGroup, static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), this, &action_debuger::updateSlider);
    connect(timer, &QTimer::timeout, this, &action_debuger::procTimer);
    client_.start();
    btnrunPos->setEnabled(false);
}

void action_debuger::procPosNameChanged(int id)
{
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->item(id - 1));

    if (pCur_PosWidget == nullptr)
    {
        cout << "empty\n";
    }

    string new_name = pCur_PosWidget->pos_name->text().toStdString();
    string old_name = pCur_PosWidget->pos_name_;
    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    if (ROBOT->get_pos_map().find(new_name) == ROBOT->get_pos_map().end())
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: " + QString::fromStdString(new_name) + " does not exist, create it?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            pCur_PosWidget->pos_name->setText(QString::fromStdString(old_name));
            return;
        }

        pCur_PosWidget->pos_name->setText(QString::fromStdString(new_name));
        robot_pos tempPos;
        tempPos.name = new_name;
        tempPos.pose_info = ROBOT->get_pos_map()[old_name].pose_info;
        ROBOT->get_pos_map()[new_name] = tempPos;
    }

    ROBOT->get_act_map()[act_name].poses[id - 1].pos_name = new_name;
}

void action_debuger::procPosTimeChanged(int id)
{
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->item(id - 1));
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    ROBOT->get_act_map()[act_name].poses[id - 1].act_time = pCur_PosWidget->pos_time->text().toInt();
}

void action_debuger::procTimer()
{
    if (client_.is_connected())
    {
        if (first_connect)
        {
            client_.regist(REMOTE_DATA, DIR_SUPPLY);
        }

        first_connect = false;
        btnrunPos->setEnabled(true);
        netstatuslab->setStyleSheet("background-color:green");
    }
    else
    {
        first_connect = true;
        btnrunPos->setEnabled(false);
        netstatuslab->setStyleSheet("background-color:red");
    }
}

void action_debuger::initActs()
{
    m_pPosListWidget->clear();
    m_pActListWidget->clear();

    for (auto &act : ROBOT->get_act_map())
    {
        m_pActListWidget->addItem(QString::fromStdString(act.second.name));
    }

    mSliderGroup->setEnabled(false);
}

void action_debuger::initPoseMap()
{
    motion_ = MOTION_BODY;
    last_motion = MOTION_BODY;
    robot_pose temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 1; i <= 6; i++)
    {
        pose_map_[static_cast<robot_motion >(i)] = temp;
    }

    for (auto &j : ROBOT->get_joint_map())
    {
        joint_degs_[j.second->jid_] = j.second->get_deg();
    }
}

void action_debuger::initStatusBar()
{
    QStatusBar *bar = statusBar();
    motionlab = new QLabel;
    valuelab = new QLabel;
    curractlab = new QLabel;
    currposlab = new QLabel;
    netstatuslab = new QLabel;
    motionlab->setMinimumWidth(60);
    valuelab->setMinimumWidth(60);
    curractlab->setMinimumWidth(60);
    currposlab->setMinimumWidth(60);
    netstatuslab->setMinimumWidth(100);
    bar->addWidget(motionlab);
    bar->addWidget(valuelab);
    bar->addWidget(curractlab);
    bar->addWidget(currposlab);
    bar->addWidget(netstatuslab);
}

void action_debuger::initJDInfo()
{
    mJDInfos.clear();
    CJointDegWidget *pJDWidget;

    QListWidgetItem *pListItem;;
    m_pJDListWidget1->clear();
    m_pJDListWidget2->clear();

    for (auto &jd : joint_degs_)
    {
        pListItem = new QListWidgetItem;
        pJDWidget = new CJointDegWidget(ROBOT->get_joint(jd.first)->name_, jd.second);
        pJDWidget->show();
        mJDInfos[ROBOT->get_joint(jd.first)->name_] = pJDWidget;

        if (ROBOT->get_joint(jd.first)->name_ == "jhead2" || ROBOT->get_joint(jd.first)->name_.find("jr") != string::npos)
        {
            m_pJDListWidget1->addItem(pListItem);
            m_pJDListWidget1->setItemWidget(pListItem, pJDWidget);
        }
        else
        {
            m_pJDListWidget2->addItem(pListItem);
            m_pJDListWidget2->setItemWidget(pListItem, pJDWidget);
        }

        pListItem->setSizeHint(QSize(pJDWidget->rect().width(), pJDWidget->rect().height()));
    }
}

void action_debuger::updateSlider(int id)
{
    motion_ = static_cast<robot_motion >(id);
    motionlab->setText(QString::fromStdString(get_name_by_motion(motion_)));

    std::vector<CKSlider *>::iterator iter = mKsliders.begin();
    (*iter)->slider->setValue(pose_map_[motion_].x / SCALE_K);
    (*iter)->nameLab->setText("X");

    if (motion_ == MOTION_HEAD)
    {
        (*iter)->nameLab->setText("Head Pitch");
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].y / SCALE_K);
    (*iter)->nameLab->setText("Y");
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND)
    {
        (*iter)->slider->setEnabled(false);
    }
    else if (motion_ == MOTION_HEAD)
    {
        (*iter)->nameLab->setText("Head Yaw");
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].z / SCALE_K);
    (*iter)->nameLab->setText("Z");
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].roll);
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].pitch);
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].yaw);
    (*iter)->slider->setEnabled(true);

    if (motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
    {
        (*iter)->slider->setEnabled(false);
    }

    last_motion = motion_;
}

float action_debuger::get_deg_from_pose(const float &ps)
{
    int value = (int)(ps / SCALE_K);
    return DEG_RANGE / SLIDER_RANGE * value;
}

bool action_debuger::turn_joint()
{
    joint_degs_[ROBOT->get_joint("jhead2")->jid_] = get_deg_from_pose(pose_map_[MOTION_HEAD].x);
    joint_degs_[ROBOT->get_joint("jhead1")->jid_] = get_deg_from_pose(pose_map_[MOTION_HEAD].y);

    Vector3d lefthand, righthand;
    righthand[0] = pose_map_[MOTION_RIGHT_HAND].x;
    righthand[2] = pose_map_[MOTION_RIGHT_HAND].z;
    lefthand[0] = pose_map_[MOTION_LEFT_HAND].x;
    lefthand[2] = pose_map_[MOTION_LEFT_HAND].z;

    transform_matrix body_mat, leftfoot_mat, rightfoot_mat;
    body_mat = ROBOT->get_body_mat_from_pose(pose_map_[MOTION_BODY]);
    leftfoot_mat = ROBOT->get_foot_mat_from_pose(pose_map_[MOTION_LEFT_FOOT], true);
    rightfoot_mat = ROBOT->get_foot_mat_from_pose(pose_map_[MOTION_RIGHT_FOOT], false);

    vector<double> degs;

    if (ROBOT->leg_inverse_kinematics(body_mat, leftfoot_mat, degs, true))
    {
        joint_degs_[ROBOT->get_joint("jlhip3")->jid_] = rad2deg(degs[0]);
        joint_degs_[ROBOT->get_joint("jlhip2")->jid_] = rad2deg(degs[1]);
        joint_degs_[ROBOT->get_joint("jlhip1")->jid_] = rad2deg(degs[2]);
        joint_degs_[ROBOT->get_joint("jlknee")->jid_] = rad2deg(degs[3]);
        joint_degs_[ROBOT->get_joint("jlankle2")->jid_] = rad2deg(degs[4]);
        joint_degs_[ROBOT->get_joint("jlankle1")->jid_] = rad2deg(degs[5]);
    }
    else
    {
        return false;
    }

    if (ROBOT->leg_inverse_kinematics(body_mat, rightfoot_mat, degs, false))
    {
        joint_degs_[ROBOT->get_joint("jrhip3")->jid_] = rad2deg(degs[0]);
        joint_degs_[ROBOT->get_joint("jrhip2")->jid_] = rad2deg(degs[1]);
        joint_degs_[ROBOT->get_joint("jrhip1")->jid_] = rad2deg(degs[2]);
        joint_degs_[ROBOT->get_joint("jrknee")->jid_] = rad2deg(degs[3]);
        joint_degs_[ROBOT->get_joint("jrankle2")->jid_] = rad2deg(degs[4]);
        joint_degs_[ROBOT->get_joint("jrankle1")->jid_] = rad2deg(degs[5]);
    }
    else
    {
        return false;
    }

    if (ROBOT->arm_inverse_kinematics(lefthand, degs))
    {
        joint_degs_[ROBOT->get_joint("jlshoulder1")->jid_] = rad2deg(degs[0]);
        joint_degs_[ROBOT->get_joint("jlelbow")->jid_] = -rad2deg(degs[2]);
    }
    else
    {
        return false;
    }

    if (ROBOT->arm_inverse_kinematics(righthand, degs))
    {
        joint_degs_[ROBOT->get_joint("jrshoulder1")->jid_] = rad2deg(degs[0]);
        joint_degs_[ROBOT->get_joint("jrelbow")->jid_] = rad2deg(degs[2]);
    }
    else
    {
        return false;
    }

    robot_gl_->turn_joint(joint_degs_);
    updateJDInfo();
    return true;
}

void action_debuger::procX(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].x;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].x = v;

    if (leftArm->isChecked())
    {
        pose_map_[MOTION_LEFT_HAND].x = v;
    }

    if (rightArm->isChecked())
    {
        pose_map_[MOTION_RIGHT_HAND].x = v;
    }

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].x = v;
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].x = v;
    }

    if (!turn_joint())
    {
        pose_map_[motion_].x = old_value;
    }
}

void action_debuger::procY(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].y;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].y = v;

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].y = v;
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].y = v;
    }

    if (!turn_joint())
    {
        pose_map_[motion_].y = old_value;
    }
}

void action_debuger::procZ(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].z;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].z = v;

    if (leftArm->isChecked())
    {
        pose_map_[MOTION_LEFT_HAND].z = v;
    }

    if (rightArm->isChecked())
    {
        pose_map_[MOTION_RIGHT_HAND].z = v;
    }

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].z = v;
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].z = v;
    }

    if (!turn_joint())
    {
        pose_map_[motion_].z = old_value;
    }
}

void action_debuger::procRoll(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].roll;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].roll = ((float) value);

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].roll = ((float) value);
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].roll = ((float) value);
    }

    if (!turn_joint())
    {
        pose_map_[motion_].roll = old_value;
    }
}

void action_debuger::procPitch(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].pitch;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].pitch = ((float) value);

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].pitch = ((float) value);
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].pitch = ((float) value);
    }

    if (!turn_joint())
    {
        pose_map_[motion_].pitch = old_value;
    }
}

void action_debuger::procYaw(int value)
{
    if (m_pPosListWidget->currentRow() == last_pos_id && motion_ == last_motion)
    {
        pos_saved = false;
    }

    float old_value = pose_map_[motion_].yaw;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].yaw = ((float) value);

    if (leftFoot->isChecked())
    {
        pose_map_[MOTION_LEFT_FOOT].yaw = ((float) value);
    }

    if (rightFoot->isChecked())
    {
        pose_map_[MOTION_RIGHT_FOOT].yaw = ((float) value);
    }

    if (!turn_joint())
    {
        pose_map_[motion_].yaw = old_value;
    }
}

void action_debuger::updateJDInfo()
{
    for (auto &jd : joint_degs_)
    {
        mJDInfos[ROBOT->get_joint(jd.first)->name_]->deg->setText(QString::number(jd.second, 'f', 2));
    }

    update();
}

void action_debuger::updatePosList(string act_name)
{
    robot_act act = ROBOT->get_act_map()[act_name];
    QListWidgetItem *pListItem;
    CPosListWidget *pPosWidget;
    int id = 0;
    m_pPosListWidget->clear();

    for (auto &pos : act.poses)
    {
        id++;
        pListItem = new QListWidgetItem;
        pPosWidget = new CPosListWidget(id, pos.pos_name, pos.act_time);
        pPosWidget->show();
        connect(pPosWidget, &CPosListWidget::nameChanged, this, &action_debuger::procPosNameChanged);
        connect(pPosWidget, &CPosListWidget::timeChanged, this, &action_debuger::procPosTimeChanged);
        m_pPosListWidget->addItem(pListItem);
        m_pPosListWidget->setItemWidget(pListItem, pPosWidget);
        pListItem->setSizeHint(QSize(pPosWidget->rect().width(), pPosWidget->rect().height()));
    }
}

void action_debuger::procActSelect(QListWidgetItem *item)
{
    if (!pos_saved)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos have not been saved, continue?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            m_pActListWidget->setCurrentRow(last_act_id);
            return;
        }
    }

    pos_saved = true;
    last_pos_id = MAX_POS_ID;
    last_act_id = m_pActListWidget->currentRow();
    curractlab->setText(item->text());
    currposlab->setText("");
    string name = m_pActListWidget->currentItem()->text().toStdString();
    updatePosList(name);
    mSliderGroup->setEnabled(false);
}

void action_debuger::procPosSelect(QListWidgetItem *item)
{
    if (!pos_saved)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos have not been saved, continue?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            m_pPosListWidget->setCurrentRow(last_pos_id);
            return;
        }
    }

    pos_saved = true;
    CPosListWidget *pPosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(item);
    string pos_name = pPosWidget->pos_name_;
    currposlab->setText(QString::fromStdString(pos_name));
    valuelab->setText("");
    pose_map_ = ROBOT->get_pos_map()[pos_name].pose_info;
    updateSlider(static_cast<int>(motion_));
    turn_joint();
    mSliderGroup->setEnabled(true);
    last_pos_id = m_pPosListWidget->currentRow();
}

void action_debuger::procButtonInsertPosFront()
{
    if (m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }

    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    bool ok;
    string new_pos_name = QInputDialog::getText(this, tr("act name"), tr("input act name:"),
                          QLineEdit::Normal, nullptr, &ok).toStdString();

    if (!ok || new_pos_name.empty())
    {
        return;
    }

    bool exist = false;

    if (ROBOT->get_pos_map().find(new_pos_name) != ROBOT->get_pos_map().end())
    {
        exist = true;
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: " + QString::fromStdString(new_pos_name) + "  exists, use it?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            return;
        }
    }

    robot_pos pos;
    robot_one_pos one_pos;
    one_pos.act_time = 100;
    one_pos.pos_name = new_pos_name;

    pos.name = new_pos_name;
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    string pos_name = pCur_PosWidget->pos_name_;
    pos.pose_info = ROBOT->get_pos_map()[pos_name].pose_info;
    int id = pCur_PosWidget->m_id->text().toInt();
    ROBOT->get_act_map()[act_name].poses.insert(ROBOT->get_act_map()[act_name].poses.begin() + id - 1, one_pos);

    if (!exist)
    {
        ROBOT->get_pos_map()[new_pos_name] = pos;
    }

    updatePosList(act_name);
}

void action_debuger::procButtonInsertPosBack()
{
    if (m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }

    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    bool ok;
    string new_pos_name = QInputDialog::getText(this, tr("pos name"), tr("input pos name:"), QLineEdit::Normal, nullptr, &ok).toStdString();

    if (!ok || new_pos_name.empty())
    {
        return;
    }

    bool exist = false;

    if (ROBOT->get_pos_map().find(new_pos_name) != ROBOT->get_pos_map().end())
    {
        exist = true;
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: " + QString::fromStdString(new_pos_name) + "  exists, use it?",
                                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

        if (reply != QMessageBox::StandardButton::Yes)
        {
            return;
        }
    }

    robot_pos pos;
    robot_one_pos one_pos;
    one_pos.act_time = 100;
    one_pos.pos_name = new_pos_name;

    pos.name = new_pos_name;

    if (m_pPosListWidget->currentItem() == nullptr)
    {
        robot_pose pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        for (int i = 1; i <= 6; i++)
        {
            pos.pose_info[static_cast<robot_motion >(i)] = pose;
        }

        ROBOT->get_act_map()[act_name].poses.push_back(one_pos);
    }
    else
    {
        CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
        string pos_name = pCur_PosWidget->pos_name_;
        pos.pose_info = ROBOT->get_pos_map()[pos_name].pose_info;
        int id = pCur_PosWidget->m_id->text().toInt();
        ROBOT->get_act_map()[act_name].poses.insert(ROBOT->get_act_map()[act_name].poses.begin() + id, one_pos);
    }

    if (!exist)
    {
        ROBOT->get_pos_map()[new_pos_name] = pos;
    }

    updatePosList(act_name);
}

void action_debuger::procButtonDeletePos()
{
    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    int id = pCur_PosWidget->m_id->text().toInt();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    ROBOT->get_act_map()[act_name].poses.erase(ROBOT->get_act_map()[act_name].poses.begin() + id - 1);
    removeUnusedPos();
    updatePosList(act_name);
}

void action_debuger::procButtonSavePos()
{
    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    string pos_name = pCur_PosWidget->pos_name_;
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "current pos is: " + QString::fromStdString(pos_name) + ", save?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

    if (reply != QMessageBox::StandardButton::Yes)
    {
        return;
    }

    ROBOT->get_pos_map()[pos_name].pose_info = pose_map_;
    pos_saved = true;
}

void action_debuger::procButtonDeleteAction()
{
    if (m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }

    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    auto iter = ROBOT->get_act_map().begin();

    while (iter != ROBOT->get_act_map().end())
    {
        if (iter->first == act_name)
        {
            ROBOT->get_act_map().erase(iter);
            break;
        }

        iter++;
    }

    removeUnusedPos();
    initActs();
}

void action_debuger::procButtonSaveAction()
{
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "all action data will be written into file, save?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

    if (reply != QMessageBox::StandardButton::Yes)
    {
        return;
    }

    action_parser::save(CONF->action_file(), ROBOT->get_act_map(), ROBOT->get_pos_map());
}

void action_debuger::procButtonAddAction()
{
    bool ok;
    string name = QInputDialog::getText(this, tr("act name"), tr("input act name:"), QLineEdit::Normal, nullptr, &ok).toStdString();

    if (name.empty())
    {
        return;
    }

    if (ok)
    {
        if (ROBOT->get_act_map().find(name) != ROBOT->get_act_map().end())
        {
            QMessageBox::warning(this, "Waring", "the name had been used", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            return;
        }

        robot_act act;
        act.name = name;
        ROBOT->get_act_map()[name] = act;
        initActs();
    }
}

void action_debuger::removeUnusedPos()
{
    bool fd;
    auto p_iter = ROBOT->get_pos_map().begin();

    while (p_iter != ROBOT->get_pos_map().end())
    {
        fd = false;

        for (auto &act : ROBOT->get_act_map())
        {
            for (auto &p : act.second.poses)
            {
                if (p.pos_name == p_iter->first)
                {
                    fd = true;
                    break;
                }
            }

            if (fd)
            {
                break;
            }
        }

        if (!fd)
        {
            p_iter = ROBOT->get_pos_map().erase(p_iter);
        }
        else
        {
            p_iter++;
        }
    }
}

void action_debuger::procButtonRunPos()
{
    if (m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    int id = pCur_PosWidget->m_id->text().toInt();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    robot_act act = ROBOT->get_act_map()[act_name];
    robot_pos pos;
    tcp_command cmd;
    cmd.type = REMOTE_DATA;
    cmd.data.clear();
    remote_data_type rtp = ACT_DATA;
    cmd.data.append((char *)(&rtp), enum_size);
    unsigned int size = enum_size;

    for (int i = 0; i < id; i++)
    {
        cmd.data.append((char *)(&(act.poses[i].act_time)), int_size);
        size += int_size;
        pos = ROBOT->get_pos_map()[act.poses[i].pos_name];

        for (auto nmm : name_motion_map)
        {
            if (nmm.second != MOTION_HEAD && nmm.second != MOTION_NONE)
            {
                cmd.data.append((char *)(&(pos.pose_info[nmm.second])), sizeof(robot_pose));
                size += sizeof(robot_pose);
            }
        }
    }

    cmd.size = size;
    client_.write(cmd);
}

void action_debuger::procButtonWalkRemote()
{
    walk_remote *window = new walk_remote(client_, net_info, this);
    window->show();
}

void action_debuger::procButtonJointRevise()
{
    joint_revise *window = new joint_revise(client_, net_info, this);
    window->show();
}

void action_debuger::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "write action data into file?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

    if (reply == QMessageBox::StandardButton::Yes)
    {
        action_parser::save(CONF->action_file(), ROBOT->get_act_map(), ROBOT->get_pos_map());
    }

    client_.stop();
}